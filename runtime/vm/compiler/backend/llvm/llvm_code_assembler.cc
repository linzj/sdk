#include "vm/compiler/backend/llvm/llvm_code_assembler.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include <elf.h>

#include "vm/bitmap.h"
#include "vm/compiler/aot/dispatch_table_generator.h"
#include "vm/compiler/backend/flow_graph.h"
#include "vm/compiler/backend/flow_graph_compiler.h"
#include "vm/compiler/backend/il.h"
#include "vm/compiler/backend/llvm/compiler_state.h"
#include "vm/compiler/backend/llvm/dwarf_info.h"
#include "vm/compiler/backend/llvm/exception_table_parser.h"
#include "vm/compiler/backend/llvm/stack_map_info.h"
#include "vm/compiler/backend/llvm/stack_maps.h"
#include "vm/compiler/backend/llvm/target_specific.h"

namespace dart {
DECLARE_FLAG(bool, source_lines);
namespace dart_llvm {
static inline uint32_t DART_USED Extract32(uint32_t value,
                                           int start,
                                           int length) {
  assert(start >= 0 && length > 0 && length <= 32 - start);
  return (value >> start) & (~0U >> (32 - length));
}

static inline int32_t DART_USED Sextract32(uint32_t value,
                                           int start,
                                           int length) {
  assert(start >= 0 && length > 0 && length <= 32 - start);
  /* Note that this implementation relies on right shift of signed
     * integers being an arithmetic shift.
     */
  return ((int32_t)(value << (32 - length - start))) >> (32 - length);
}

#if defined(TARGET_ARCH_ARM)
#include "vm/compiler/backend/llvm/llvm_code_assembler_arm.cc"
#elif defined(TARGET_ARCH_ARM64)
#include "vm/compiler/backend/llvm/llvm_code_assembler_arm64.cc"
#else
#error unsupported arch
#endif
CodeAssembler::CodeAssembler(FlowGraphCompiler* compiler)
    : compiler_(compiler), arch_impl_(new ArchImpl) {
  llvm_compiler_state_ = std::move(compiler->flow_graph().ReleaseLLVMState());
  EMASSERT(compiler_state().code_section_list_.size() == 1);
  const ByteBuffer& code_buffer = *compiler_state().code_section_list_.back();
  code_start_ = code_buffer.data();
  code_size_ = bytes_left_ = code_buffer.size();
  compiler->InitCompiler();
  GraphEntryInstr* graph_entry = compiler->flow_graph().graph_entry();
  exception_extend_id_ = graph_entry->catch_entries().length();
}

CodeAssembler::~CodeAssembler() {}

void CodeAssembler::AssembleCode() {
  if (compiler().TryIntrinsify() && compiler().skip_body_compilation()) {
    return;
  }
  PrepareExceptionTable();
  PrepareInstrActions();
  SetupEntryCodeRange();
  for (auto& p : action_map_) {
    unsigned end = p.first;
    auto& actions = p.second;
    if (offset_ != end) {
      EMASSERT(end > offset_);
      assembler().EmitRange(code_start_ + offset_, end - offset_);
      bytes_left_ -= end - offset_;
      offset_ = end;
    }
    for (auto& action : actions)
      action();
  }
  if (bytes_left_ != 0) {
    assembler().EmitRange(code_start_ + offset_, bytes_left_);
  }
  EndLastInstr();
  EmitCP();
  EmitExceptionHandler();
}

FlowGraphCompiler& CodeAssembler::compiler() {
  return *compiler_;
}
const CompilerState& CodeAssembler::compiler_state() {
  return *llvm_compiler_state_;
}
compiler::Assembler& CodeAssembler::assembler() {
  return *compiler().assembler();
}

void CodeAssembler::PrepareExceptionTable() {
  if (!compiler_state().exception_table_) return;
  ExceptionTableParser parser(compiler_state().exception_table_->data(),
                              compiler_state().exception_table_->size());
  exception_tuples_ = std::move(parser.CallSiteHandlerPairs());
}

void CodeAssembler::PrepareInstrActions() {
  PrepareDwarfAction();
  PrepareStackMapAction();
  PrepareLoadCPAction();
}

void CodeAssembler::PrepareDwarfAction() {
  DwarfLineMapper mapper;
  mapper.Process(compiler_state().dwarf_line_->data());
  auto& debug_instrs_ = compiler_state().debug_instrs_;
  for (auto& p : mapper.GetMap()) {
    if (p.second <= 1) continue;
    unsigned pc_offset = p.first;
    unsigned index = p.second - 2;
    Instruction* instr = debug_instrs_[index];
    auto func = [this, instr]() -> size_t {
      EndLastInstr();
      compiler().code_source_map_builder_->StartInliningInterval(
          assembler().CodeSize(), instr->inlining_id());
      if (FLAG_code_comments || FLAG_disassemble ||
          FLAG_disassemble_optimized) {
        // This flag is not defined as global.
        if (FLAG_source_lines) {
          compiler().EmitSourceLine(instr);
        }
        compiler().EmitComment(instr);
      }
      compiler().BeginCodeSourceRange();
      compiler().StatsBegin(instr);
      last_instr_ = instr;
      if (instr->IsReturn()) {
        ReturnInstr* return_instr = instr->AsReturn();
        if (return_instr->yield_index() !=
            RawPcDescriptors::kInvalidYieldIndex) {
          compiler().EmitYieldPositionMetadata(return_instr->token_pos(),
                                               return_instr->yield_index());
        }
      }
      if (instr->IsStoreStaticField()) {
        compiler().used_static_fields().Add(
            &instr->AsStoreStaticField()->field());
      }
      if (instr->IsLoadStaticField()) {
        compiler().used_static_fields().Add(
            &instr->AsLoadStaticField()->StaticField());
      }
      return static_cast<size_t>(0);
    };
    AddAction(pc_offset, WrapAction(func));
  }
}

static bool IsLiveOutHasNoneGPRegister(
    const std::vector<StackMaps::LiveOut>& liveouts) {
  for (auto& e : liveouts) {
    if (e.dwarfReg > kDwarfGenernalRegEnd) {
      return true;
    }
  }
  return false;
};

static Register DwarfToRegister(DWARFRegister dwarf) {
#if defined(TARGET_ARCH_ARM)
  return static_cast<Register>(dwarf);
#elif defined(TARGET_ARCH_ARM64)
  return static_cast<Register>(dwarf);
#else
#error unsupported arch
#endif
}
static Register GetCallTargetRegFromRecordForRegType(
    const StackMaps::Record& record) {
  auto& location = record.locations[0];
  EMASSERT(location.kind == StackMaps::Location::Register);
  return DwarfToRegister(location.dwarfReg);
}

void CodeAssembler::PrepareStackMapAction() {
  if (!compiler_state().stackMapsSection_) return;
  StackMaps sm;
  DataView dv(compiler_state().stackMapsSection_->data());
  sm.parse(&dv);
  slot_count_ = sm.stackSize() / compiler::target::kWordSize;
  if (slot_count_ != 0) {
    EMASSERT(slot_count_ >= 2);
    // must minus 2 slot for fp and return address.
    slot_count_ -= 2;
  }
  compiler().flow_graph().graph_entry()->set_spill_slot_count(slot_count_);

  auto rm = sm.computeRecordMap();
  const StackMapInfoMap& stack_map_info_map =
      compiler_state().stack_map_info_map_;
  for (auto& item : rm) {
    EMASSERT(item.second.size() == 1);
    const auto& record = item.second.front();
    uint32_t instruction_offset = item.first;
    auto stack_map_info_found = stack_map_info_map.find(record.patchpointID);
    EMASSERT(stack_map_info_found != stack_map_info_map.end());
    const StackMapInfo* stack_map_info = stack_map_info_found->second.get();
    EMASSERT(stack_map_info->GetType() == StackMapInfoType::kCallInfo);
    const CallSiteInfo* call_site_info =
        static_cast<const CallSiteInfo*>(stack_map_info);
    std::function<void()> f;
    switch (call_site_info->type()) {
      case CallSiteInfo::CallTargetType::kReg:
        f = WrapAction([this, call_site_info, record]() {
          CallWithCallReg(call_site_info,
                          GetCallTargetRegFromRecordForRegType(record));
          AddMetaData(call_site_info, record);
          if (call_site_info->code() != nullptr)
            compiler().AddStubCallTarget(*call_site_info->code());
          if (call_site_info->return_on_stack_pos() != -1) {
            assembler().LoadMemoryValue(CallingConventions::kReturnReg, SP,
                                        call_site_info->return_on_stack_pos() *
                                            compiler::target::kWordSize);
          }
          return call_site_info->instr_size();
        });
        break;
      case CallSiteInfo::CallTargetType::kCallRelative:
        f = WrapAction([this, call_site_info, record]() {
          assembler().GenerateUnRelocatedPcRelativeCall();
          compiler().AddPcRelativeCallTarget(*call_site_info->target(),
                                             call_site_info->entry_kind());
          AddMetaData(call_site_info, record);
          return call_site_info->instr_size();
        });
        break;
      case CallSiteInfo::CallTargetType::kStubRelative:
        f = WrapAction([this, call_site_info, record]() {
          bool should_use_fp_stub = IsLiveOutHasNoneGPRegister(record.liveOuts);
          assembler().GenerateUnRelocatedPcRelativeCall();
          const Code* stub = call_site_info->code();
          if (should_use_fp_stub && call_site_info->fpu_code())
            stub = call_site_info->fpu_code();
          compiler().AddPcRelativeCallStubTarget(*stub);
          AddMetaData(call_site_info, record);
          return call_site_info->instr_size();
        });
        break;
      case CallSiteInfo::CallTargetType::kThreadOffset:
        f = WrapAction([this, call_site_info, record]() {
          bool should_use_fp_thread_offset =
              IsLiveOutHasNoneGPRegister(record.liveOuts);
          intptr_t thread_offset = call_site_info->thread_offset();
          if (should_use_fp_thread_offset &&
              call_site_info->fpu_thread_offset())
            thread_offset = call_site_info->fpu_thread_offset();
          assembler().LoadMemoryValue(kCallTargetReg, THR, thread_offset);
          CallWithCallReg(call_site_info, kCallTargetReg);
          AddMetaData(call_site_info, record);
          return call_site_info->instr_size();
        });
        break;
      case CallSiteInfo::CallTargetType::kNative:
        f = WrapAction([this, call_site_info, record]() {
          GenerateNativeCall(call_site_info);
          AddMetaData(call_site_info, record);
          assembler().LoadMemoryValue(CallingConventions::kReturnReg, SP,
                                      call_site_info->return_on_stack_pos() *
                                          compiler::target::kWordSize);
          return call_site_info->instr_size();
        });
        break;
      case CallSiteInfo::CallTargetType::kPatchable:
        f = WrapAction([this, call_site_info, record]() {
          GeneratePatchableCall(call_site_info);
          AddMetaData(call_site_info, record);
          return call_site_info->instr_size();
        });
        break;
      case CallSiteInfo::CallTargetType::kCCall:
        f = WrapAction([this, call_site_info]() {
          GenerateCCall(call_site_info);
          return call_site_info->instr_size();
        });
        break;
      default:
        UNREACHABLE();
    }
    AddAction(instruction_offset, f);
  }
}

template <typename T>
std::function<void()> CodeAssembler::WrapAction(T f) {
  return [this, f]() {
    intptr_t before_action = assembler().CodeSize();
    size_t expected_adv = f();
    intptr_t after_action = assembler().CodeSize();
    intptr_t adv = after_action - before_action;
    EMASSERT(expected_adv == static_cast<size_t>(adv));
    offset_ += adv;
    bytes_left_ -= adv;
  };
}

void CodeAssembler::AddMetaData(const CallSiteInfo* call_site_info,
                                const StackMaps::Record& r) {
  if (UNLIKELY(!call_site_info->needs_metadata())) return;
  intptr_t try_index = CollectExceptionInfo(call_site_info);
  compiler().AddDescriptor(call_site_info->kind(), assembler().CodeSize(),
                           call_site_info->deopt_id(),
                           call_site_info->token_pos(), try_index);
  if (try_index != kInvalidTryIndex) {
    compiler().catch_entry_moves_maps_builder_->NewMapping(
        assembler().CodeSize());
    compiler().catch_entry_moves_maps_builder_->EndMapping();
  }
  RecordSafePoint(call_site_info, r);
}

intptr_t CodeAssembler::CollectExceptionInfo(
    const CallSiteInfo* call_site_info) {
  if (exception_tuples_.empty()) {
    EMASSERT(call_site_info->try_index() == kInvalidTryIndex);
    return kInvalidTryIndex;
  }
  if (call_site_info->try_index() == kInvalidTryIndex) return kInvalidTryIndex;
  intptr_t try_index = call_site_info->try_index();
  intptr_t origin_try_index = try_index;
#if 0
  printf("printing exception_tuples_\n");
  for (auto& t : exception_tuples_) {
    printf("0x%x %d 0x%x\n", std::get<0>(t), std::get<1>(t), std::get<2>(t));
  }
#endif
  auto found =
      std::lower_bound(exception_tuples_.begin(), exception_tuples_.end(),
                       assembler().CodeSize(),
                       [](const std::tuple<int, int, int>& lhs, intptr_t rhs) {
                         return std::get<0>(lhs) < rhs;
                       });
  if (found == exception_tuples_.end() ||
      std::get<0>(*found) != assembler().CodeSize())
    found -= 1;
  EMASSERT(assembler().CodeSize() >= std::get<0>(*found));
  EMASSERT(assembler().CodeSize() <= std::get<0>(*found) + std::get<1>(*found));
  int exception_block_off = std::get<2>(*found);
  int branch_offset = BranchOffset(code_start_ + exception_block_off);
  if (branch_offset != -1) exception_block_off += branch_offset;
  auto crf_found = emited_idx_crf_.find(exception_block_off);
  if (crf_found != emited_idx_crf_.end()) return crf_found->second;
  auto exception_map_found = exception_map_.find(try_index);
  if (exception_map_found != exception_map_.end() &&
      std::get<0>(exception_map_found->second) !=
          static_cast<size_t>(exception_block_off)) {
    // need a extened try index;

    intptr_t extended_try_index = exception_extend_id_++;
    try_index = extended_try_index;
#if 0
    printf("handling extened exception for function:%s\n",
           compiler()
               .flow_graph()
               .parsed_function()
               .function()
               .ToFullyQualifiedCString());
#endif
  }
  exception_map_.emplace(
      try_index, std::make_tuple(exception_block_off, origin_try_index));
  emited_idx_crf_.emplace(exception_block_off, try_index);
  return try_index;
}

void CodeAssembler::RecordSafePoint(const CallSiteInfo* call_site_info,
                                    const StackMaps::Record& record) {
  if (UNLIKELY(call_site_info->is_tailcall())) return;
  BitmapBuilder* builder = new BitmapBuilder();
  builder->SetLength(slot_count_);

  for (auto& location : record.locations) {
    if (location.kind != StackMaps::Location::Indirect) continue;
    // only understand stack slot
    int index;
    if (location.dwarfReg == kDwarfSPForSafePoint) {
      EMASSERT(location.offset >= 0);
      EMASSERT(location.offset / compiler::target::kWordSize <
               static_cast<int>(slot_count_));
      index = slot_count_ - 1 - location.offset / compiler::target::kWordSize;
    } else {
      EMASSERT(location.dwarfReg == kDwarfFP);
      if (location.offset >= 0) continue;
      index = -location.offset / compiler::target::kWordSize - kDwarfFPFixup;
    }
    builder->Set(index, true);
  }
  // set up parameters
  size_t top_of_stack = slot_count_;
  int64_t parameter_bits = call_site_info->parameter_bits();
  int valid_bits = call_site_info->valid_bits();
  int i;

  for (i = 0; i < valid_bits; ++i) {
    if ((parameter_bits & (1ULL << i)) == 0) continue;
    int index = top_of_stack - i - 1;
    builder->Set(index, true);
  }
  compiler().compressed_stackmaps_builder()->AddEntry(assembler().CodeSize(),
                                                      builder, slot_count_);
}

void CodeAssembler::EmitExceptionHandler() {
  if (exception_map_.empty()) return;
  GraphEntryInstr* graph_entry = compiler().flow_graph().graph_entry();
  for (auto& p : exception_map_) {
    intptr_t try_index = p.first;
    intptr_t origin_try_index;
    size_t exception_block_off;
    std::tie(exception_block_off, origin_try_index) = p.second;
    CatchBlockEntryInstr* catch_block =
        graph_entry->GetCatchEntry(origin_try_index);
    compiler().AddExceptionHandler(
        try_index, catch_block->try_index(), exception_block_off,
        catch_block->is_generated(), catch_block->catch_handler_types(),
        catch_block->needs_stacktrace());
  }
}

void CodeAssembler::EndLastInstr() {
  if (!last_instr_) return;

  if (last_instr_->IsDispatchTableCall()) {
    DispatchTableCallInstr* dispatch_table_call_instr =
        last_instr_->AsDispatchTableCall();
    if (dispatch_table_call_instr->selector()->called_on_null &&
        !dispatch_table_call_instr->selector()->on_null_interface) {
      Value* receiver = dispatch_table_call_instr->ArgumentValueAt(
          dispatch_table_call_instr->FirstArgIndex());
      if (receiver->Type()->is_nullable()) {
        const String& function_name = String::ZoneHandle(
            dispatch_table_call_instr->interface_target().name());
        compiler().AddNullCheck(dispatch_table_call_instr->token_pos(),
                                function_name);
      }
    }
    compiler().AddDispatchTableCallTarget(
        dispatch_table_call_instr->selector());
  }
  compiler().StatsEnd(last_instr_);
  compiler().EndCodeSourceRange(last_instr_->token_pos());
}

void CodeAssembler::AddAction(size_t pc_offset, std::function<void()> action) {
  auto found = action_map_.find(pc_offset);
  if (LIKELY(found == action_map_.end())) {
    action_map_.emplace(pc_offset, std::vector<std::function<void()>>{action});
  } else {
    found->second.emplace_back(action);
  }
}

void CodeAssembler::SetupEntryCodeRange() {
  const FlowGraph& flow_graph = compiler().flow_graph();
  GraphEntryInstr* graph_entry = flow_graph.graph_entry();
  FunctionEntryInstr* normal_entry = graph_entry->normal_entry();
  last_instr_ = normal_entry;
}

void CodeAssembler::GenerateCCall(const CallSiteInfo* call_site_info) {
  assembler().CallRuntime(*call_site_info->runtime_entry(),
                          call_site_info->stack_parameter_count());
}
}  // namespace dart_llvm
}  // namespace dart
#endif
