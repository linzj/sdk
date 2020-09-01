#include "vm/compiler/backend/llvm/ir_translator.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include <unordered_map>

#include "vm/compiler/aot/precompiler.h"
#include "vm/compiler/assembler/object_pool_builder.h"
#include "vm/compiler/backend/il_printer.h"
#include "vm/compiler/backend/llvm/compile.h"
#include "vm/compiler/backend/llvm/compiler_state.h"
#include "vm/compiler/backend/llvm/initialize_llvm.h"
#include "vm/compiler/backend/llvm/liveness_analysis.h"
#include "vm/compiler/backend/llvm/llvm_log.h"
#include "vm/compiler/backend/llvm/output.h"
#include "vm/compiler/backend/llvm/stack_map_info.h"
#include "vm/compiler/backend/llvm/target_specific.h"
#include "vm/compiler/runtime_api.h"
#include "vm/dispatch_table.h"
#include "vm/object_store.h"
#include "vm/type_testing_stubs.h"

namespace dart {
namespace dart_llvm {
namespace {
class AnonImpl;
class Label;
class AssemblerResolver;
struct NotMergedPhiDesc {
  BlockEntryInstr* pred;
  int value;
  LValue phi;
};

enum class ValueType { LLVMValue };

struct ValueDesc {
  ValueType type;
  union {
    LValue llvm_value;
  };
};

using ExceptionBlockLiveInValueMap = std::unordered_map<int, LValue>;

struct IRTranslatorBlockImpl {
  std::vector<NotMergedPhiDesc> not_merged_phis;
  std::unordered_map<int, ValueDesc> values_;

  LBasicBlock native_bb = nullptr;
  LBasicBlock continuation = nullptr;

  // exception vals
  LValue exception_val = nullptr;
  LValue stacktrace_val = nullptr;
  LValue exception_pp_val = nullptr;
  std::unordered_map<int, LValue> exception_params;
  bool request_exception_env_ = false;

  AnonImpl* anon_impl;
  int try_index = kInvalidTryIndex;

  bool started = false;
  bool ended = false;
  bool need_merge = false;
  bool exception_block = false;

  inline void SetLLVMValue(int ssa_id, LValue value) {
    values_[ssa_id] = {ValueType::LLVMValue, {value}};
  }

  inline void SetValue(int nid, const ValueDesc& value) {
    values_[nid] = value;
  }

  LValue GetLLVMValue(int ssa_id);

  LValue GetLLVMValue(Value* v) {
    return GetLLVMValue(v->definition()->ssa_temp_index());
  }

  bool IsDefined(Definition* def) {
    auto found = values_.find(def->ssa_temp_index());
    return found != values_.end();
  }

#if 0
  LValue GetLLVMValue(Definition* d) {
    return GetLLVMValue(d->ssa_temp_index());
  }
#endif

  const ValueDesc& GetValue(int nid) const {
    auto found = values_.find(nid);
    EMASSERT(found != values_.end());
    return found->second;
  }

  const ValueDesc& GetValue(Value* v) const {
    return GetValue(v->definition()->ssa_temp_index());
  }

  inline std::unordered_map<int, ValueDesc>& values() { return values_; }

  inline void StartTranslate() {
    EMASSERT(!started);
    EMASSERT(!ended);
    started = true;
  }

  inline void EndTranslate() {
    EMASSERT(started);
    EMASSERT(!ended);
    ended = true;
  }
};

class ValueInterceptor {
 public:
  virtual ~ValueInterceptor() = default;
  // return true means intercepted, and anon impl must not record this value.
  // return false means not intercepted, and anon impl must record this value.
  virtual bool SetLLVMValue(int, LValue) = 0;
  virtual LValue GetLLVMValue(int) = 0;
};

class AnonImpl {
 public:
  AnonImpl() = default;
  ~AnonImpl() = default;
  LBasicBlock GetNativeBB(BlockEntryInstr* bb);
  LBasicBlock EnsureNativeBB(BlockEntryInstr* bb);
  IRTranslatorBlockImpl* GetTranslatorBlockImpl(BlockEntryInstr* bb);
  LBasicBlock GetNativeBBContinuation(BlockEntryInstr* bb);
  bool IsBBStartedToTranslate(BlockEntryInstr* bb);
  bool IsBBEndedToTranslate(BlockEntryInstr* bb);
  void StartTranslate(BlockEntryInstr* bb);
  void MergePredecessors(BlockEntryInstr* bb);
  // When called, the current builder position must point to the end of catch block's native bb.
  bool AllPredecessorStarted(BlockEntryInstr* bb, BlockEntryInstr** ref_pred);
  void End();
  void EndCurrentBlock();
  void ProcessPhiWorkList();
  void BuildPhiAndPushToWorkList(BlockEntryInstr* bb,
                                 BlockEntryInstr* ref_pred);
  // Phis
  LValue EnsurePhiInput(BlockEntryInstr* pred, int index, LType type);
  LValue EnsurePhiInputAndPosition(BlockEntryInstr* pred,
                                   int index,
                                   LType type);
  // Exception handling
  LBasicBlock EnsureNativeCatchBlock(int try_index);
  IRTranslatorBlockImpl* GetCatchBlockImpl(CatchBlockEntryInstr* instr);
  void CollectExceptionVars();
  // Debug & line info.
  void SetCurrentInstr(Instruction*);
  Instruction* CurrentDebugInstr();
  // Access to memory.
  LValue BuildAccessPointer(LValue base, LValue offset, Representation rep);
  LValue BuildAccessPointer(LValue base, LValue offset, LType type);
  // Calls
  int NextPatchPoint();
  void SubmitStackMap(std::unique_ptr<StackMapInfo> info);
  void PushArgument(LValue v);
  LValue GenerateCall(
      Instruction* instr,
      TokenPosition token_pos,
      intptr_t deopt_id,
      const Code& stub,
      RawPcDescriptors::Kind kind,
      size_t stack_argument_count,
      const std::vector<std::pair<Register, LValue>>& gp_parameters) {
    return GenerateCall(instr, token_pos, deopt_id, nullptr, stub, nullptr,
                        kind, stack_argument_count, gp_parameters);
  }

  LValue GenerateCall(
      Instruction*,
      TokenPosition token_pos,
      intptr_t deopt_id,
      const char* csr,
      const Code& stub,
      const Code* fpu_stub,
      RawPcDescriptors::Kind kind,
      size_t stack_argument_count,
      const std::vector<std::pair<Register, LValue>>& gp_parameters);
  LValue GenerateRuntimeCall(Instruction*,
                             TokenPosition token_pos,
                             intptr_t deopt_id,
                             const RuntimeEntry& entry,
                             intptr_t argument_count,
                             bool return_on_stack = true);
  LValue GenerateStaticCall(
      Instruction*,
      int ssa_index,
      const std::vector<LValue>& arguments,
      intptr_t deopt_id,
      TokenPosition token_pos,
      const Function& function,
      ArgumentsInfo args_info,
      const ICData& ic_data_in,
      Code::EntryKind entry_kind = Code::EntryKind::kNormal);
  LValue GenerateMegamorphicInstanceCall(Instruction* instr,
                                         const String& name,
                                         const Array& arguments_descriptor,
                                         intptr_t deopt_id,
                                         TokenPosition token_pos);
  // InstanceOf
#if 0
  RawSubtypeTestCache* GenerateInlineInstanceof(
      LValue value,
      LValue instantiator_type_arguments, 
      LValue function_type_arguments,
      AssemblerResolver& resolver,
      TokenPosition token_pos,
      const AbstractType& type,
      Label& is_instance_lbl,
      Label& is_not_instance_lbl);
#endif
  // Types
  LType GetMachineRepresentationType(Representation);
  LType DeduceReturnType();
  LValue TaggedToWord(LValue v);
  LValue WordToTagged(LValue v);
  LValue EnsureBoolean(LValue v);
  LValue EnsureIntPtr(LValue v);
  LValue EnsureInt32(LValue v);
  LValue SmiTag(LValue v);
  LValue SmiUntag(LValue v);
  LValue IsSmi(LValue v);
  LValue BooleanToObject(LValue boolean);
  LValue LoadClassId(LValue o);
  LValue CompareClassId(LValue o, intptr_t clsid);
  LValue CompareObject(LValue, const Object&);
  LValue BitcastDoubleToInt64(LValue);
  LValue BitcastInt64ToDouble(LValue);
  LValue BitcastFloatToInt32(LValue);
  LValue BitcastInt32ToFloat(LValue);

  // expect
  LValue ExpectTrue(LValue);
  LValue ExpectFalse(LValue);

  // Value helpers for current bb
  LValue GetLLVMValue(int ssa_id);

  LValue GetLLVMValue(Value* v) {
    return GetLLVMValue(v->definition()->ssa_temp_index());
  }
  LValue GetLLVMValue(Definition* d) {
    return GetLLVMValue(d->ssa_temp_index());
  }

  LValue GetPPValue() { return GetLLVMValue(liveness().GetPPValueSSAIdx()); }
  void SetLLVMValue(int ssa_id, LValue v);
  void SetLLVMValue(Definition* d, LValue v);
  void SetLazyValue(Definition*);
  // MaterializeDef
  LValue MaterializeDef(Definition* d);
  bool IsLazyValue(int ssa_index);

  // Load Store
  compiler::ObjectPoolBuilder& object_pool_builder() {
    return *precompiler_->global_object_pool_builder();
  }
  LValue LoadFieldFromOffset(LValue base, int offset, LType type);
  LValue LoadFieldFromOffset(LValue base, int offset);
  LValue LoadSmiFieldFromOffset(LValue base, int offset);
  LValue LoadFromOffset(LValue base,
                        int offset,
                        LType type,
                        bool invariant = false);
  LValue LoadObject(const Object& object, bool is_unique = false);
  LValue LoadObjectFromPool(intptr_t offset);
  LValue GetDispatchTable();
  LValue GetNull();
  LValue GetBarrierMask();

  void StoreToOffset(LValue base, int offset, LValue v);
  void StoreToOffset(LValue base, LValue offset, LValue v);
  void StoreIntoObject(Instruction* instruction,
                       LValue object,  // Object we are storing into.
                       LValue dest,    // Where we are storing into.
                       LValue value,   // Value we are storing.
                       compiler::Assembler::CanBeSmi can_value_be_smi =
                           compiler::Assembler::kValueCanBeSmi);
  void StoreIntoArray(Instruction* instruction,
                      LValue object,  // Object we are storing into.
                      LValue dest,    // Where we are storing into.
                      LValue value,   // Value we are storing.
                      compiler::Assembler::CanBeSmi can_value_be_smi =
                          compiler::Assembler::kValueCanBeSmi);
  // Allocate
  LValue TryAllocate(AssemblerResolver& resolver,
                     Label& failure,
                     intptr_t cid,
                     intptr_t instance_size);
  LValue BoxInteger32(Instruction*, LValue value, bool fit, Representation rep);

  // Basic blocks continuation
  LBasicBlock NewBasicBlock(const char* fmt, ...);
  void SetCurrentBlockContinuation(LBasicBlock continuation);
  LBasicBlock GetCurrentBlockContinuation();

  // Target dependent
  bool support_integer_div() const;
  void AdjustInstrSizeForLoadFromConsecutiveAddress(size_t& instr_size,
                                                    intptr_t offset) const;

  void AdjustInstrSizeForLoad(size_t& instr_size, intptr_t offset) const;
  inline CompilerState& compiler_state() { return *compiler_state_; }
  inline LivenessAnalysis& liveness() { return *liveness_analysis_; }
  inline Output& output() { return *output_; }
  inline IRTranslatorBlockImpl* current_bb_impl() { return current_bb_impl_; }
  inline BlockEntryInstr* current_bb() { return current_bb_; }
  inline FlowGraph* flow_graph() { return flow_graph_; }
  inline Thread* thread() { return thread_; }
  inline ObjectStore* object_store() {
    return flow_graph()->isolate()->object_store();
  }

  // CSR
  const char* GetSharedStubCSR();
  const char* GetCCallCSR();
  const char* GetBoxInt64SharedStubCSR();
  // classes
  const Class& mint_class() {
    return Class::ZoneHandle(object_store()->mint_class());
  }

  const Class& double_class() {
    return Class::ZoneHandle(object_store()->double_class());
  }

  const Class& float32x4_class() {
    return Class::ZoneHandle(object_store()->float32x4_class());
  }

  const Class& float64x2_class() {
    return Class::ZoneHandle(object_store()->float64x2_class());
  }

  const Class& int32x4_class() {
    return Class::ZoneHandle(object_store()->int32x4_class());
  }

  Zone* zone() { return flow_graph_->zone(); }
  void set_exception_occured() { exception_occured_ = true; }

  BlockEntryInstr* current_bb_;
  IRTranslatorBlockImpl* current_bb_impl_;
  FlowGraph* flow_graph_;
  Precompiler* precompiler_;
  std::unique_ptr<CompilerState> compiler_state_;
  std::unique_ptr<LivenessAnalysis> liveness_analysis_;
  std::unique_ptr<Output> output_;
  std::unordered_map<BlockEntryInstr*, IRTranslatorBlockImpl> block_map_;
  StackMapInfoMap stack_map_info_map_;
  std::vector<BlockEntryInstr*> phi_rebuild_worklist_;
  //Exception handling
  std::vector<LBasicBlock> catch_blocks_;
  // lazy values
  std::unordered_map<int, Definition*> lazy_values_;

  // Value intercept
  ValueInterceptor* value_interceptor_ = nullptr;

  // debug lines
  std::vector<Instruction*> debug_instrs_;
  // Calls
  std::vector<LValue> pushed_arguments_;
  Thread* thread_;
  Environment* pending_deoptimization_env_ = nullptr;
  int patch_point_id_ = 0;
  bool visited_function_entry_ = false;
  bool exception_occured_ = false;
  DISALLOW_COPY_AND_ASSIGN(AnonImpl);
};

class ModifiedValuesMergeHelper : public ValueInterceptor {
 public:
  explicit ModifiedValuesMergeHelper(AnonImpl&);
  ~ModifiedValuesMergeHelper() override;
  void AssignBlockInitialValues(LBasicBlock);
  void InitialValuesInBlock(LBasicBlock);
  void InitialValuesInBlockToEmpty();
  void AssignModifiedValueToMerge(LBasicBlock);
  void MergeModified();
  void Enable();
  void Disable();
  AnonImpl& impl() { return impl_; }
  Output& output() { return impl_.output(); }

 private:
  bool SetLLVMValue(int ssa_id, LValue index) override;
  LValue GetLLVMValue(int ssa_id) override;

  AnonImpl& impl_;
  BitVector* values_modified_;
  std::unordered_map<LBasicBlock, std::unordered_map<int, LValue>>
      initial_modified_values_lookup_;
  std::unordered_map<int, LValue> current_modified_;
  std::unordered_map<LBasicBlock, std::unordered_map<int, LValue>>
      modified_values_to_merge_;
  ValueInterceptor* old_value_interceptor_;
};

class ContinuationResolver {
 protected:
  ContinuationResolver(AnonImpl& impl, int ssa_id);
  ~ContinuationResolver() = default;
  inline AnonImpl& impl() { return impl_; }
  inline Output& output() { return impl().output(); }
  inline BlockEntryInstr* current_bb() { return impl().current_bb(); }
  inline int ssa_id() const { return ssa_id_; }

 private:
  AnonImpl& impl_;
  int ssa_id_;
  DISALLOW_COPY_AND_ASSIGN(ContinuationResolver);
};

class DiamondContinuationResolver : public ContinuationResolver {
 public:
  DiamondContinuationResolver(AnonImpl& impl, int ssa_id);
  ~DiamondContinuationResolver() = default;
  DiamondContinuationResolver& BuildCmp(std::function<LValue()>);
  DiamondContinuationResolver& BuildLeft(std::function<LValue()>);
  DiamondContinuationResolver& BuildRight(std::function<LValue()>);
  LValue End();
  void BranchToOtherLeafOnCondition(LValue cond);
  LBasicBlock left_block() { return blocks_[0]; }
  LBasicBlock right_block() { return blocks_[1]; }
  void LeftHint() { hint_ = -1; }
  void RightHint() { hint_ = 1; }

 private:
  ModifiedValuesMergeHelper modified_value_helper_;
  LBasicBlock blocks_[3];
  LValue values_[2];
  // 0 no hint
  // -1 left hint
  // 1 right hint
  int hint_;
  int building_block_index_;
};

class CallResolver : public ContinuationResolver {
 public:
  struct CallResolverParameter {
    CallResolverParameter(Instruction*, std::unique_ptr<CallSiteInfo>);
    ~CallResolverParameter() = default;
    Instruction* call_instruction;
    std::unique_ptr<CallSiteInfo> callsite_info;
    DISALLOW_COPY_AND_ASSIGN(CallResolverParameter);
  };
  explicit CallResolver(AnonImpl& impl,
                        int ssa_id,
                        CallResolverParameter& call_resolver_parameter);
  ~CallResolver() = default;
  void SetGParameter(int reg, LValue);
  void SetCallTarget(LValue call_target) { call_target_ = call_target; }
  void AddStackParameter(LValue);
  LValue GetStackParameter(size_t i);
  LValue BuildCall();
  void set_callee_save_register(const char* callee_save_registers) {
    callee_save_registers_ = callee_save_registers;
  }

 private:
  void GenerateStatePointFunction();
  void ExtractCallInfo();
  void EmitCall();
  void EmitTailCall();
  void EmitRelocatesIfNeeded();
  void EmitPatchPoint();
  void EmitExceptionVars();
  void EmitCSR();
  bool need_invoke() { return tail_call_ == false && catch_block_ != nullptr; }

  CallResolverParameter& call_resolver_parameter_;

  // state in the middle.
  LValue statepoint_function_ = nullptr;
  LType callee_type_ = nullptr;
  LType return_type_ = nullptr;
  LValue call_value_ = nullptr;
  LValue statepoint_value_ = nullptr;
  LValue call_target_ = nullptr;
  LBasicBlock from_block_ = nullptr;
  // exception blocks
  IRTranslatorBlockImpl* catch_block_impl_ = nullptr;
  LBasicBlock exception_native_bb_ = nullptr;
  LBasicBlock continuation_block_ = nullptr;
  CatchBlockEntryInstr* catch_block_ = nullptr;
  BitVector* call_live_out_ = nullptr;
  const char* callee_save_registers_ = nullptr;
  std::unordered_map<int /* ssa_index */, int /* where */> gc_desc_map_;
  std::unordered_map<int /* var index */, int /* where */>
      exception_gc_desc_map_;
  std::vector<LValue> parameters_;
  int parameter_current_slot_ = 0;
  int patchid_ = 0;
  int pp_value_at_state_point_ = 0;
  bool tail_call_ = false;
  DISALLOW_COPY_AND_ASSIGN(CallResolver);
};

class BoxAllocationSlowPath {
 public:
  BoxAllocationSlowPath(Instruction* instruction,
                        const Class& cls,
                        AnonImpl& impl);
  LValue Allocate();
  AnonImpl& impl() { return impl_; }
  Output& output() { return *impl_.output_; }

 private:
  LValue BuildSlowPath();
  Instruction* instruction_;
  const Class& cls_;
  AnonImpl& impl_;
  DISALLOW_COPY_AND_ASSIGN(BoxAllocationSlowPath);
};

class ComparisonResolver : public FlowGraphVisitor {
 public:
  explicit ComparisonResolver(AnonImpl& impl);
  ~ComparisonResolver() = default;
  AnonImpl& impl() { return impl_; }
  Output& output() { return *impl_.output_; }
  LValue result() {
    EMASSERT(result_ != nullptr);
    return result_;
  }

 private:
  void VisitStrictCompare(StrictCompareInstr* instr) override;
  void VisitEqualityCompare(EqualityCompareInstr* instr) override;
  void VisitCheckedSmiComparison(CheckedSmiComparisonInstr* instr) override;
  void VisitCaseInsensitiveCompare(CaseInsensitiveCompareInstr* instr) override;
  void VisitRelationalOp(RelationalOpInstr* instr) override;
  void VisitDoubleTestOp(DoubleTestOpInstr* instr) override;

  AnonImpl& impl_;
  LValue result_;
  DISALLOW_COPY_AND_ASSIGN(ComparisonResolver);
};

class ReturnRepresentationDeducer : public FlowGraphVisitor {
 public:
  explicit ReturnRepresentationDeducer(AnonImpl& impl);
  ~ReturnRepresentationDeducer() = default;
  Representation Deduce();

 private:
  void VisitReturn(ReturnInstr*) override;
  Representation rep_;
  DISALLOW_COPY_AND_ASSIGN(ReturnRepresentationDeducer);
};

class Label {
 public:
  Label(const char* fmt, ...);

 private:
  void InitBBIfNeeded(AnonImpl& impl);
  LBasicBlock basic_block() { return bb_; }
  friend class AssemblerResolver;
  std::string name_;
  LBasicBlock bb_;
};

class AssemblerResolver : private ModifiedValuesMergeHelper {
 public:
  AssemblerResolver(AnonImpl&);
  ~AssemblerResolver() override;

  void Branch(Label&);
  void BranchIf(LValue cond, Label&);
  void BranchIfNot(LValue cond, Label&);
  void Bind(Label&);
  LValue End();
  void GotoMerge();
  void GotoMergeIf(LValue cond);
  void GotoMergeIfNot(LValue cond);
  void GotoMergeWithValue(LValue v);
  void GotoMergeWithValueIf(LValue cond, LValue v);
  void GotoMergeWithValueIfNot(LValue cond, LValue v);

 private:
  LBasicBlock merge_;
  // Must an ordered list with merge values
  // The unordered_map modified_values_to_merge_ from ModifiedValuesMergeHelper
  // is unordered thus unusable.
  std::vector<LBasicBlock> merge_blocks_;
  std::vector<LValue> merge_values_;
};

// Switch to other block.
class BlockScope {
 public:
  BlockScope(AnonImpl& impl, BlockEntryInstr*);
  ~BlockScope();

 private:
  BlockEntryInstr* saved_;
  AnonImpl& impl_;
  DISALLOW_COPY_AND_ASSIGN(BlockScope);
};

LValue IRTranslatorBlockImpl::GetLLVMValue(int ssa_id) {
  auto found = values_.find(ssa_id);
  EMASSERT(found != values_.end());
  EMASSERT(found->second.type == ValueType::LLVMValue);
  EMASSERT(found->second.llvm_value);
  return found->second.llvm_value;
}

IRTranslatorBlockImpl* AnonImpl::GetTranslatorBlockImpl(BlockEntryInstr* bb) {
  auto& ref = block_map_[bb];
  if (ref.anon_impl == nullptr) {
    // init.
    ref.anon_impl = this;
  }
  return &ref;
}

LBasicBlock AnonImpl::EnsureNativeBB(BlockEntryInstr* bb) {
  IRTranslatorBlockImpl* impl = GetTranslatorBlockImpl(bb);
  if (impl->native_bb) return impl->native_bb;
  char buf[256];
  snprintf(buf, 256, "B%d", static_cast<int>(bb->block_id()));
  LBasicBlock native_bb = output_->appendBasicBlock(buf);
  impl->native_bb = native_bb;
  impl->continuation = native_bb;
  return native_bb;
}

LBasicBlock AnonImpl::GetNativeBB(BlockEntryInstr* bb) {
  return GetTranslatorBlockImpl(bb)->native_bb;
}

LBasicBlock AnonImpl::GetNativeBBContinuation(BlockEntryInstr* bb) {
  return GetTranslatorBlockImpl(bb)->continuation;
}

bool AnonImpl::IsBBStartedToTranslate(BlockEntryInstr* bb) {
  auto impl = GetTranslatorBlockImpl(bb);
  return impl->started;
}

bool AnonImpl::IsBBEndedToTranslate(BlockEntryInstr* bb) {
  auto impl = GetTranslatorBlockImpl(bb);
  return impl->ended;
}

void AnonImpl::StartTranslate(BlockEntryInstr* bb) {
  current_bb_ = bb;
  current_bb_impl_ = GetTranslatorBlockImpl(bb);
  current_bb_impl()->StartTranslate();
  EnsureNativeBB(bb);
  output_->positionToBBEnd(GetNativeBB(bb));
}

void AnonImpl::MergePredecessors(BlockEntryInstr* bb) {
  intptr_t predecessor_count = bb->PredecessorCount();
  if (predecessor_count == 0) return;
  IRTranslatorBlockImpl* block_impl = GetTranslatorBlockImpl(bb);
  EMASSERT(!block_impl->exception_block);
  if (predecessor_count == 1) {
    // Don't use phi if only one predecessor.
    BlockEntryInstr* pred = bb->PredecessorAt(0);
    IRTranslatorBlockImpl* pred_block_impl = GetTranslatorBlockImpl(pred);
    EMASSERT(IsBBStartedToTranslate(pred));
    for (BitVector::Iterator it(liveness().GetLiveInSet(bb)); !it.Done();
         it.Advance()) {
      int live = it.Current();
      if (IsLazyValue(live)) continue;
      auto& value = pred_block_impl->GetValue(live);
      block_impl->SetValue(live, value);
    }
    return;
  }
  BlockEntryInstr* ref_pred = nullptr;
  if (!AllPredecessorStarted(bb, &ref_pred)) {
    EMASSERT(!!ref_pred);
    BuildPhiAndPushToWorkList(bb, ref_pred);
    return;
  }
  // Use phi.
  for (BitVector::Iterator it(liveness().GetLiveInSet(bb)); !it.Done();
       it.Advance()) {
    int live = it.Current();
    if (IsLazyValue(live)) continue;
    auto& value = GetTranslatorBlockImpl(ref_pred)->GetValue(live);
    if (value.type != ValueType::LLVMValue) {
      block_impl->SetValue(live, value);
      continue;
    }
    LValue ref_value = value.llvm_value;
    LType ref_type = typeOf(ref_value);
    if (ref_type != output().tagged_type()) {
      // FIXME: Should add EMASSERT that all values are the same.
      block_impl->SetLLVMValue(live, ref_value);
      continue;
    }
    LValue phi = output().buildPhi(ref_type);
    for (intptr_t i = 0; i < predecessor_count; ++i) {
      BlockEntryInstr* pred = bb->PredecessorAt(i);
      LValue value = GetTranslatorBlockImpl(pred)->GetLLVMValue(live);
      LBasicBlock native = GetNativeBBContinuation(pred);
      addIncoming(phi, &value, &native, 1);
    }
    block_impl->SetLLVMValue(live, phi);
  }
}

bool AnonImpl::AllPredecessorStarted(BlockEntryInstr* bb,
                                     BlockEntryInstr** ref_pred) {
  bool ret_value = true;
  intptr_t predecessor_count = bb->PredecessorCount();
  for (intptr_t i = 0; i < predecessor_count; ++i) {
    BlockEntryInstr* pred = bb->PredecessorAt(i);
    if (IsBBStartedToTranslate(pred)) {
      if (!*ref_pred) *ref_pred = pred;
    } else {
      ret_value = false;
    }
  }
  return ret_value;
}

void AnonImpl::BuildPhiAndPushToWorkList(BlockEntryInstr* bb,
                                         BlockEntryInstr* ref_pred) {
  IRTranslatorBlockImpl* block_impl = GetTranslatorBlockImpl(bb);
  IRTranslatorBlockImpl* ref_pred_impl = GetTranslatorBlockImpl(ref_pred);
  for (BitVector::Iterator it(liveness().GetLiveInSet(bb)); !it.Done();
       it.Advance()) {
    int live = it.Current();
    if (IsLazyValue(live)) continue;
    const ValueDesc& value_desc = ref_pred_impl->GetValue(live);
    if (value_desc.type != ValueType::LLVMValue) {
      block_impl->SetValue(live, value_desc);
      continue;
    }
    LValue ref_value = value_desc.llvm_value;
    LType ref_type = typeOf(ref_value);
    if (ref_type != output().tagged_type()) {
      block_impl->SetLLVMValue(live, ref_value);
      continue;
    }
    LValue phi = output().buildPhi(ref_type);
    block_impl->SetLLVMValue(live, phi);
    intptr_t predecessor_count = bb->PredecessorCount();
    for (intptr_t i = 0; i < predecessor_count; ++i) {
      BlockEntryInstr* pred = bb->PredecessorAt(i);
      if (!IsBBStartedToTranslate(pred)) {
        block_impl->not_merged_phis.emplace_back();
        NotMergedPhiDesc& not_merged_phi = block_impl->not_merged_phis.back();
        not_merged_phi.phi = phi;
        not_merged_phi.value = live;
        not_merged_phi.pred = pred;
        continue;
      }
      LValue value = GetTranslatorBlockImpl(pred)->GetLLVMValue(live);
      LBasicBlock native = GetNativeBBContinuation(pred);
      addIncoming(phi, &value, &native, 1);
    }
  }
  phi_rebuild_worklist_.push_back(bb);
}

void AnonImpl::End() {
  if (current_bb_) EndCurrentBlock();
  ProcessPhiWorkList();
  output().positionToBBEnd(output().prologue());
  output().buildBr(GetNativeBB(flow_graph_->graph_entry()));
  output().finalize();
  output().EmitDebugInfo(std::move(debug_instrs_));
  output().EmitStackMapInfoMap(std::move(stack_map_info_map_));
  Compile(compiler_state());
  flow_graph_->SetLLVMCompilerState(std::move(compiler_state_));
}

void AnonImpl::EndCurrentBlock() {
  EMASSERT(current_bb_impl_->continuation != nullptr);
  GetTranslatorBlockImpl(current_bb_)->EndTranslate();
  current_bb_ = nullptr;
  current_bb_impl_ = nullptr;
  pushed_arguments_.clear();
}

void AnonImpl::ProcessPhiWorkList() {
  for (BlockEntryInstr* bb : phi_rebuild_worklist_) {
    auto impl = GetTranslatorBlockImpl(bb);
    for (auto& e : impl->not_merged_phis) {
      BlockEntryInstr* pred = e.pred;
      EMASSERT(IsBBStartedToTranslate(pred));
      LValue value = EnsurePhiInput(pred, e.value, typeOf(e.phi));
      LBasicBlock native = GetNativeBBContinuation(pred);
      addIncoming(e.phi, &value, &native, 1);
    }
    impl->not_merged_phis.clear();
  }
  phi_rebuild_worklist_.clear();
}

LValue AnonImpl::EnsurePhiInput(BlockEntryInstr* pred, int index, LType type) {
  LValue val;
  auto found_in_lazy_values = lazy_values_.find(index);
  if (found_in_lazy_values != lazy_values_.end()) {
    LValue terminator =
        LLVMGetBasicBlockTerminator(GetNativeBBContinuation(pred));
    output().positionBefore(terminator);
    BlockScope block_scope(*this, pred);
    val = MaterializeDef(found_in_lazy_values->second);
    return val;
  } else {
    val = GetTranslatorBlockImpl(pred)->GetLLVMValue(index);
  }
  LType value_type = typeOf(val);
  if (value_type == type) return val;
  LValue terminator =
      LLVMGetBasicBlockTerminator(GetNativeBBContinuation(pred));
  if ((value_type == output().repo().intPtr) &&
      (type == output().tagged_type())) {
    output().positionBefore(terminator);
    LValue ret_val = WordToTagged(val);
    return ret_val;
  }
  LLVMTypeKind value_type_kind = LLVMGetTypeKind(value_type);
  if ((LLVMPointerTypeKind == value_type_kind) &&
      (type == output().repo().intPtr)) {
    output().positionBefore(terminator);
    LValue ret_val = TaggedToWord(val);
    return ret_val;
  }
  if ((value_type == output().repo().boolean) &&
      (type == output().repo().intPtr)) {
    output().positionBefore(terminator);
    LValue ret_val = output().buildCast(LLVMZExt, val, output().repo().intPtr);
    return ret_val;
  }
  LLVMTypeKind type_kind = LLVMGetTypeKind(type);
  if ((LLVMIntegerTypeKind == value_type_kind) &&
      (value_type_kind == type_kind)) {
    // handle both integer
    output().positionBefore(terminator);
    LValue ret_val;
    if (LLVMGetIntTypeWidth(value_type) > LLVMGetIntTypeWidth(type)) {
      ret_val = output().buildCast(LLVMTrunc, val, type);
    } else {
      ret_val = output().buildCast(LLVMZExt, val, type);
    }
    return ret_val;
  }
  __builtin_trap();
}

LValue AnonImpl::EnsurePhiInputAndPosition(BlockEntryInstr* pred,
                                           int index,
                                           LType type) {
  LValue value = EnsurePhiInput(pred, index, type);
  output().positionToBBEnd(GetNativeBB(current_bb_));
  return value;
}

LValue AnonImpl::MaterializeDef(Definition* def) {
  LValue v;
  if (UnboxedConstantInstr* unboxed = def->AsUnboxedConstant()) {
    const Object& object = unboxed->value();
    if (object.IsDouble()) {
      const Double& n = Double::Cast(object);
      v = output().constDouble(n.value());
    } else if (object.IsSmi()) {
      intptr_t smi_value = Smi::Cast(object).Value();
      switch (unboxed->representation()) {
        case kUnboxedUint32:
        case kUnboxedInt32:
          v = output().constInt32(smi_value);
          break;
        case kUnboxedInt64:
          v = output().constInt64(smi_value);
          break;
        default:
          UNREACHABLE();
      }
    } else if (object.IsMint()) {
      v = output().constInt64(Mint::Cast(object).value());
    } else {
      LLVMLOGE(
          "MaterializeDef: UnboxedConstantInstr failed to interpret: unknown "
          "type: %s",
          object.ToCString());
      UNREACHABLE();
    }
  } else if (ConstantInstr* constant_object = def->AsConstant()) {
    const Object& object = constant_object->value();
    if (compiler::IsSameObject(compiler::NullObject(), object))
      v = GetNull();
    else
      v = LoadObject(object);
  } else {
    LLVMLOGE("MaterializeDef: unknown def: %s\n", def->ToCString());
    UNREACHABLE();
  }
  return v;
}

bool AnonImpl::IsLazyValue(int ssa_index) {
  return lazy_values_.find(ssa_index) != lazy_values_.end();
}

LBasicBlock AnonImpl::EnsureNativeCatchBlock(int try_index) {
  char buf[256];
  LBasicBlock native_bb;
  if (catch_blocks_.size() < static_cast<size_t>(try_index + 1))
    catch_blocks_.resize(try_index + 1);
  if ((native_bb = catch_blocks_[try_index]) == nullptr) {
    snprintf(buf, 256, "B_catch_block_%d", try_index);
    native_bb = output().appendBasicBlock(buf);
    catch_blocks_[try_index] = native_bb;
  }
  return native_bb;
}

IRTranslatorBlockImpl* AnonImpl::GetCatchBlockImpl(
    CatchBlockEntryInstr* instr) {
  IRTranslatorBlockImpl* block_impl = GetTranslatorBlockImpl(instr);
  EMASSERT(block_impl->native_bb);
  return block_impl;
}

void AnonImpl::CollectExceptionVars() {
  int try_index = 0;
  LBasicBlock prologue = output().getInsertionBlock();
  while (CatchBlockEntryInstr* instr =
             flow_graph()->graph_entry()->GetCatchEntry(try_index++)) {
    IRTranslatorBlockImpl* block_impl = GetTranslatorBlockImpl(instr);
    LBasicBlock native_bb = EnsureNativeCatchBlock(instr->catch_try_index());
    block_impl->native_bb = native_bb;
    block_impl->continuation = native_bb;
    block_impl->exception_block = true;

    output().positionToBBEnd(native_bb);
    block_impl->exception_val = output().buildPhi(output().tagged_type());
    block_impl->stacktrace_val = output().buildPhi(output().tagged_type());
    block_impl->exception_pp_val = output().buildPhi(output().tagged_type());
    for (Definition* def : *instr->initial_definitions()) {
      ParameterInstr* param = def->AsParameter();
      if (!param) continue;
      int param_index = param->index();
      if (param_index < flow_graph()->num_direct_parameters()) continue;

      LValue phi = output().buildPhi(output().tagged_type());
      block_impl->exception_params.emplace(param_index, phi);
      block_impl->request_exception_env_ = true;
    }
    block_impl->SetLLVMValue(liveness().GetPPValueSSAIdx(),
                             block_impl->exception_pp_val);
  }

  output().positionToBBEnd(prologue);
}

void AnonImpl::SetCurrentInstr(Instruction* instr) {
  // Setup Debug line.
  debug_instrs_.emplace_back(instr);
  // line one is too special, ignore it.
  output().setDebugInfo(debug_instrs_.size() + 1, nullptr);
  // Setup pending_deoptimization_env_
  pending_deoptimization_env_ = instr->env();
}

Instruction* AnonImpl::CurrentDebugInstr() {
  return debug_instrs_.back();
}

LValue AnonImpl::BuildAccessPointer(LValue base,
                                    LValue offset,
                                    Representation rep) {
  return BuildAccessPointer(base, offset,
                            pointerType(GetMachineRepresentationType(rep)));
}

LValue AnonImpl::BuildAccessPointer(LValue base, LValue offset, LType type) {
  LLVMTypeKind kind = LLVMGetTypeKind(typeOf(base));
  if (kind == LLVMIntegerTypeKind) {
    base = output().buildCast(LLVMIntToPtr, base, output().repo().ref8);
  }
  LValue pointer = output().buildGEPWithByteOffset(base, offset, type);
  return pointer;
}

int AnonImpl::NextPatchPoint() {
  return patch_point_id_++;
}

void AnonImpl::SubmitStackMap(std::unique_ptr<StackMapInfo> info) {
  int patchid = info->patchid();
  auto where = stack_map_info_map_.emplace(patchid, std::move(info));
  EMASSERT(where.second && "Submit overlapped patch id");
}

void AnonImpl::PushArgument(LValue v) {
  pushed_arguments_.emplace_back(v);
}

LValue AnonImpl::GenerateCall(
    Instruction* instr,
    TokenPosition token_pos,
    intptr_t deopt_id,
    const char* csr,
    const Code& stub,
    const Code* fpu_stub,
    RawPcDescriptors::Kind kind,
    size_t stack_argument_count,
    const std::vector<std::pair<Register, LValue>>& gp_parameters) {
  if (!stub.InVMIsolateHeap()) {
    std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
    callsite_info->set_type(CallSiteInfo::CallTargetType::kStubRelative);
    callsite_info->set_token_pos(token_pos);
    callsite_info->set_code(&stub);
    callsite_info->set_fpu_code(fpu_stub);
    callsite_info->set_kind(kind);
    callsite_info->set_deopt_id(deopt_id);
    callsite_info->set_stack_parameter_count(stack_argument_count);
    callsite_info->set_instr_size(kCallInstrSize);
    CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
    CallResolver resolver(*this, -1, param);
    for (size_t i = 0; i < stack_argument_count; ++i) {
      LValue argument = pushed_arguments_.back();
      pushed_arguments_.pop_back();
      resolver.AddStackParameter(argument);
    }
    EMASSERT(fpu_stub == nullptr || csr != nullptr);
    resolver.set_callee_save_register(csr);
    for (auto p : gp_parameters) {
      resolver.SetGParameter(Output::RegisterToParameterLoc(p.first), p.second);
    }
    return resolver.BuildCall();
  } else {
    const int32_t offset = compiler::target::ObjectPool::element_offset(
        object_pool_builder().FindObject(
            compiler::ToObject(stub),
            compiler::ObjectPoolBuilderEntry::kNotPatchable));

    LValue code_object = LoadObjectFromPool(offset);
    LValue entry_gep = output().buildGEPWithByteOffset(
        code_object,
        output().constIntPtr(
            compiler::target::Code::entry_point_offset(CodeEntryKind::kNormal) -
            kHeapObjectTag),
        pointerType(output().repo().ref8));
    LValue entry = output().buildLoad(entry_gep);

    std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
    callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
    callsite_info->set_token_pos(token_pos);
    callsite_info->set_kind(kind);
    callsite_info->set_stack_parameter_count(stack_argument_count);
    callsite_info->set_instr_size(kCallInstrSize);
    callsite_info->set_code(&stub);
    CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
    CallResolver resolver(*this, -1, param);
    resolver.SetGParameter(Output::RegisterToParameterLoc(CODE_REG),
                           code_object);
    resolver.SetCallTarget(entry);
    for (size_t i = 0; i < stack_argument_count; ++i) {
      LValue argument = pushed_arguments_.back();
      pushed_arguments_.pop_back();
      resolver.AddStackParameter(argument);
    }
    for (auto p : gp_parameters) {
      resolver.SetGParameter(Output::RegisterToParameterLoc(p.first), p.second);
    }
    return resolver.BuildCall();
  }
}

LValue AnonImpl::GenerateRuntimeCall(Instruction* instr,
                                     TokenPosition token_pos,
                                     intptr_t deopt_id,
                                     const RuntimeEntry& runtime_entry,
                                     intptr_t argument_count,
                                     bool return_on_stack) {
  EMASSERT(!runtime_entry.is_leaf());
  LValue runtime_entry_point =
      LoadFromOffset(output().thread(),
                     compiler::target::Thread::OffsetFromThread(&runtime_entry),
                     pointerType(output().repo().ref8), true);
  LValue target = LoadFromOffset(
      output().thread(),
      compiler::target::Thread::call_to_runtime_entry_point_offset(),
      pointerType(output().repo().ref8), true);
  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
  callsite_info->set_token_pos(token_pos);
  callsite_info->set_deopt_id(deopt_id);
  if (return_on_stack) callsite_info->set_return_on_stack_pos(argument_count);
  callsite_info->set_stack_parameter_count(argument_count);
  callsite_info->set_instr_size(return_on_stack ? kCallReturnOnStackInstrSize
                                                : kCallInstrSize);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(*this, -1, param);
  resolver.SetCallTarget(target);
  resolver.SetGParameter(Output::RegisterToParameterLoc(kRuntimeCallEntryReg),
                         runtime_entry_point);
  resolver.SetGParameter(
      Output::RegisterToParameterLoc(kRuntimeCallArgCountReg),
      output().constIntPtr(argument_count));
  // add stack parameters.
  EMASSERT(static_cast<intptr_t>(pushed_arguments_.size()) >= argument_count);
  for (intptr_t i = 0; i < argument_count; ++i) {
    LValue argument = pushed_arguments_.back();
    pushed_arguments_.pop_back();
    resolver.AddStackParameter(argument);
  }
  if (return_on_stack) {
    resolver.AddStackParameter(GetNull());
  }
  return resolver.BuildCall();
}

LValue AnonImpl::GenerateStaticCall(Instruction* instr,
                                    int ssa_index,
                                    const std::vector<LValue>& arguments,
                                    intptr_t deopt_id,
                                    TokenPosition token_pos,
                                    const Function& function,
                                    ArgumentsInfo args_info,
                                    const ICData& ic_data_in,
                                    Code::EntryKind entry_kind) {
  const ICData& ic_data = ICData::ZoneHandle(ic_data_in.Original());
  const Array& arguments_descriptor = Array::ZoneHandle(
      zone(), ic_data.IsNull() ? args_info.ToArgumentsDescriptor()
                               : ic_data.arguments_descriptor());
  LValue argument_desc_val = LLVMGetUndef(output().tagged_type());
  if (function.HasOptionalParameters() || function.IsGeneric()) {
    argument_desc_val = LoadObject(arguments_descriptor);
  } else {
    if (!(FLAG_precompiled_mode && FLAG_use_bare_instructions)) {
      argument_desc_val = output().constTagged(0);
    }
  }
  int argument_count = args_info.count_with_type_args;
  EMASSERT(argument_count == static_cast<int>(arguments.size()));

  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kCallRelative);
  callsite_info->set_token_pos(token_pos);
  callsite_info->set_deopt_id(deopt_id);
  callsite_info->set_stack_parameter_count(argument_count);
  callsite_info->set_target(&function);
  callsite_info->set_entry_kind(entry_kind);
  callsite_info->set_instr_size(kCallInstrSize);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(*this, ssa_index, param);
  resolver.SetGParameter(Output::RegisterToParameterLoc(ARGS_DESC_REG),
                         argument_desc_val);
  // setup stack parameters
  for (int i = argument_count - 1; i >= 0; --i) {
    LValue param = arguments[i];
    resolver.AddStackParameter(param);
  }
  return resolver.BuildCall();
}

LValue AnonImpl::GenerateMegamorphicInstanceCall(
    Instruction* instr,
    const String& name,
    const Array& arguments_descriptor,
    intptr_t deopt_id,
    TokenPosition token_pos) {
  const ArgumentsDescriptor args_desc(arguments_descriptor);
  int argument_count = args_desc.Count();
  EMASSERT(argument_count >= static_cast<int>(pushed_arguments_.size()));
  const MegamorphicCache& cache = MegamorphicCache::ZoneHandle(
      zone(),
      MegamorphicCacheTable::Lookup(thread(), name, arguments_descriptor));
  LValue cache_value = LoadObject(cache);
  LValue entry = LoadFromOffset(
      output().thread(),
      compiler::target::Thread::megamorphic_call_checked_entry_offset(),
      pointerType(output().repo().ref8), true);

  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
  callsite_info->set_token_pos(token_pos);
  callsite_info->set_deopt_id(deopt_id);
  callsite_info->set_stack_parameter_count(argument_count);
  callsite_info->set_instr_size(kCallInstrSize);

  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(*this, -1, param);
  resolver.SetCallTarget(entry);
  for (int i = 0; i < argument_count; ++i) {
    LValue argument = pushed_arguments_.back();
    pushed_arguments_.pop_back();
    resolver.AddStackParameter(argument);
  }
  LValue receiver = resolver.GetStackParameter((argument_count - 1));
  resolver.SetGParameter(Output::RegisterToParameterLoc(kReceiverReg),
                         receiver);
  resolver.SetGParameter(Output::RegisterToParameterLoc(kICReg), cache_value);
  return resolver.BuildCall();
}

LType AnonImpl::GetMachineRepresentationType(Representation rep) {
  LType type;
  switch (rep) {
    case kTagged:
      type = output().tagged_type();
      break;
    case kUnboxedDouble:
      type = output().repo().doubleType;
      break;
    case kUnboxedFloat:
      type = output().repo().floatType;
      break;
    case kUnboxedInt32:
    case kUnboxedUint32:
      type = output().repo().int32;
      break;
    case kUnboxedInt64:
      type = output().repo().int64;
      break;
    case kUnboxedInt32x4:
      type = output().repo().int32x4;
      break;
    case kUnboxedFloat32x4:
      type = output().repo().float32x4;
      break;
    case kUnboxedFloat64x2:
      type = output().repo().float64x2;
      break;
    default:
      UNREACHABLE();
  }
  return type;
}

LType AnonImpl::DeduceReturnType() {
  ReturnRepresentationDeducer deducer(*this);
  Representation rep = deducer.Deduce();
  if (rep == kNoRepresentation) {
    // must have only throw il.
    return output().tagged_type();
  }
  switch (rep) {
    case kUnboxedInt32x4:
    case kUnboxedFloat32x4:
    case kUnboxedFloat64x2:
      EMASSERT(false && "unsupported vector type");
      break;
    default:
      break;
  }
  return GetMachineRepresentationType(rep);
}

LValue AnonImpl::TaggedToWord(LValue v) {
  EMASSERT(typeOf(v) == output().tagged_type());
  return output().buildCast(LLVMPtrToInt, v, output().repo().intPtr);
}

LValue AnonImpl::WordToTagged(LValue v) {
  EMASSERT(typeOf(v) == output().repo().intPtr);
  return output().buildCast(LLVMIntToPtr, v, output().tagged_type());
}

LValue AnonImpl::EnsureBoolean(LValue v) {
  LType type = typeOf(v);
  LLVMTypeKind kind = LLVMGetTypeKind(type);
  if (kind == LLVMPointerTypeKind) v = TaggedToWord(v);
  type = typeOf(v);
  if (LLVMGetIntTypeWidth(type) == 1) return v;
  v = output().buildICmp(LLVMIntNE, v, output().repo().intPtrZero);
  return v;
}

LValue AnonImpl::EnsureIntPtr(LValue v) {
  LType type = typeOf(v);
  if (type == output().repo().intPtr) return v;
  LLVMTypeKind kind = LLVMGetTypeKind(type);
  if (kind == LLVMPointerTypeKind)
    return output().buildCast(LLVMZExt, v, output().repo().intPtr);
  return v;
}

LValue AnonImpl::EnsureInt32(LValue v) {
  LType type = typeOf(v);
  if (type == output().repo().int32) return v;
  LLVMTypeKind kind = LLVMGetTypeKind(type);
  if (kind == LLVMPointerTypeKind) {
    return output().buildCast(LLVMPtrToInt, v, output().repo().int32);
  }
  if (type == output().repo().boolean) {
    return output().buildCast(LLVMZExt, v, output().repo().int32);
  }
  EMASSERT(type == output().repo().int32);
  return v;
}

LValue AnonImpl::SmiTag(LValue v) {
  EMASSERT(typeOf(v) == output().repo().intPtr);
  v = output().buildShl(v, output().constIntPtr(kSmiTagSize));
  return WordToTagged(v);
}

LValue AnonImpl::SmiUntag(LValue v) {
  EMASSERT(typeOf(v) == output().tagged_type());
  v = TaggedToWord(v);
  return output().buildSar(v, output().constIntPtr(kSmiTagSize));
}

LValue AnonImpl::IsSmi(LValue v) {
  EMASSERT(typeOf(v) == output().tagged_type());
  LValue address = TaggedToWord(v);
  LValue and_value =
      output().buildAnd(address, output().constIntPtr(kSmiTagMask));
  return output().buildICmp(LLVMIntNE, and_value,
                            output().constIntPtr(kSmiTagMask));
}

LValue AnonImpl::BooleanToObject(LValue boolean) {
  EMASSERT(typeOf(boolean) == output().repo().boolean);
  return output().buildSelect(boolean, LoadObject(Bool::True()),
                              LoadObject(Bool::False()));
}

LValue AnonImpl::LoadClassId(LValue o) {
  const intptr_t class_id_offset =
      compiler::target::Object::tags_offset() +
      compiler::target::RawObject::kClassIdTagPos / kBitsPerByte;
  return LoadFieldFromOffset(o, class_id_offset,
                             pointerType(output().repo().int16));
}

LValue AnonImpl::CompareClassId(LValue o, intptr_t clsid) {
  LValue clsid_val = LoadClassId(o);
  clsid_val = output().buildCast(LLVMZExt, clsid_val, output().repo().intPtr);
  return output().buildICmp(LLVMIntEQ, clsid_val, output().constIntPtr(clsid));
}

LValue AnonImpl::CompareObject(LValue v, const Object& o) {
  LValue o_val = LoadObject(o);
  return output().buildICmp(LLVMIntEQ, v, o_val);
}

LValue AnonImpl::BitcastDoubleToInt64(LValue v) {
  EMASSERT(typeOf(v) == output().repo().doubleType);

  LValue storage = output().buildBitCast(output().bitcast_space(),
                                         output().repo().refDouble);
  output().buildStore(v, storage);
  LValue int64_storage =
      output().buildBitCast(output().bitcast_space(), output().repo().ref64);
  return output().buildLoad(int64_storage);
}

LValue AnonImpl::BitcastInt64ToDouble(LValue v) {
  EMASSERT(typeOf(v) == output().repo().int64);

  LValue storage =
      output().buildBitCast(output().bitcast_space(), output().repo().ref64);
  output().buildStore(v, storage);
  LValue double_storage = output().buildBitCast(output().bitcast_space(),
                                                output().repo().refDouble);
  return output().buildLoad(double_storage);
}

LValue AnonImpl::BitcastFloatToInt32(LValue v) {
  EMASSERT(typeOf(v) == output().repo().floatType);

  LValue storage =
      output().buildBitCast(output().bitcast_space(), output().repo().refFloat);
  output().buildStore(v, storage);
  LValue int32_storage =
      output().buildBitCast(output().bitcast_space(), output().repo().ref32);
  return output().buildLoad(int32_storage);
}

LValue AnonImpl::BitcastInt32ToFloat(LValue v) {
  EMASSERT(typeOf(v) == output().repo().int32);

  LValue storage =
      output().buildBitCast(output().bitcast_space(), output().repo().ref32);
  output().buildStore(v, storage);
  LValue float_storage =
      output().buildBitCast(output().bitcast_space(), output().repo().refFloat);
  return output().buildLoad(float_storage);
}

LValue AnonImpl::ExpectTrue(LValue cond) {
  EMASSERT(typeOf(cond) == output().repo().boolean);

  return output().buildCall(output().repo().expectIntrinsic(), cond,
                            output().repo().booleanTrue);
}

LValue AnonImpl::ExpectFalse(LValue cond) {
  EMASSERT(typeOf(cond) == output().repo().boolean);

  return output().buildCall(output().repo().expectIntrinsic(), cond,
                            output().repo().booleanFalse);
}

LValue AnonImpl::GetLLVMValue(int ssa_id) {
  auto found_in_lazy_values = lazy_values_.find(ssa_id);
  if (found_in_lazy_values != lazy_values_.end()) {
    LValue v = MaterializeDef(found_in_lazy_values->second);
    return v;
  }
  if (value_interceptor_) {
    LValue v = value_interceptor_->GetLLVMValue(ssa_id);
    if (v) return v;
  }
  return current_bb_impl()->GetLLVMValue(ssa_id);
}

void AnonImpl::SetLLVMValue(int ssa_id, LValue val) {
  if (value_interceptor_) {
    if (value_interceptor_->SetLLVMValue(ssa_id, val)) return;
  }
  current_bb_impl()->SetLLVMValue(ssa_id, val);
}

void AnonImpl::SetLLVMValue(Definition* d, LValue val) {
  SetLLVMValue(d->ssa_temp_index(), val);
  if (d->HasPairRepresentation())
    SetLLVMValue(d->ssa_temp_index() + 1,
                 LLVMGetUndef(output().repo().boolean));
}

void AnonImpl::SetLazyValue(Definition* d) {
  lazy_values_.emplace(d->ssa_temp_index(), d);
}

LValue AnonImpl::LoadFieldFromOffset(LValue base, int offset, LType type) {
  LValue gep = output().buildGEPWithByteOffset(
      base, output().constIntPtr(offset - kHeapObjectTag), type);
  return output().buildLoad(gep);
}

LValue AnonImpl::LoadFieldFromOffset(LValue base, int offset) {
  return LoadFieldFromOffset(base, offset, pointerType(output().tagged_type()));
}

LValue AnonImpl::LoadSmiFieldFromOffset(LValue base, int offset) {
  LValue gep = output().buildGEPWithByteOffset(
      base, output().constIntPtr(offset - kHeapObjectTag),
      pointerType(output().tagged_type()));
  return output().buildLoadSmi(gep);
}

LValue AnonImpl::LoadFromOffset(LValue base,
                                int offset,
                                LType type,
                                bool invariant) {
  LValue gep =
      output().buildGEPWithByteOffset(base, output().constIntPtr(offset), type);
  if (LIKELY(!invariant))
    return output().buildLoad(gep);
  else
    return output().buildInvariantLoad(gep);
}

LValue AnonImpl::LoadObject(const Object& object, bool is_unique) {
  intptr_t offset = 0;
  if (compiler::target::CanLoadFromThread(object, &offset)) {
    // Load common VM constants from the thread. This works also in places where
    // no constant pool is set up (e.g. intrinsic code).
    LValue gep = output().buildGEPWithByteOffset(
        output().thread(), output().constIntPtr(offset),
        pointerType(output().tagged_type()));
    bool invariant_load = false;
    if (compiler::IsSameObject(compiler::TrueObject(), object))
      invariant_load = true;
    else if (compiler::IsSameObject(compiler::FalseObject(), object))
      invariant_load = true;

    if (!invariant_load)
      return output().buildLoad(gep);
    else
      return output().buildInvariantLoad(gep);
  } else if (compiler::target::IsSmi(object)) {
    // Relocation doesn't apply to Smis.
    return WordToTagged(output().constIntPtr(
        static_cast<intptr_t>(compiler::target::ToRawSmi(object))));
  } else {
    // Make sure that class CallPattern is able to decode this load from the
    // object pool.
    EMASSERT(FLAG_use_bare_instructions);
    const auto index = is_unique ? object_pool_builder().AddObject(object)
                                 : object_pool_builder().FindObject(object);
    const int32_t offset = compiler::target::ObjectPool::element_offset(index);
    return LoadObjectFromPool(offset);
  }
}

LValue AnonImpl::GetDispatchTable() {
#if defined(TARGET_SUPPORT_DISPATCH_TABLE_REG)
  return output().GetRegisterParameter(DISPATCH_TABLE_REG);
#else
  return LoadFromOffset(output().thread(),
                        compiler::target::Thread::dispatch_table_array_offset(),
                        pointerType(output().repo().ref8), true);
#endif
}

LValue AnonImpl::GetNull() {
#if defined(TARGET_SUPPORT_NULL_OBJECT_REG)
  return output().GetRegisterParameter(NULL_REG);
#else
  return LoadObject(Object::null_object());
#endif
}

LValue AnonImpl::GetBarrierMask() {
  LValue barrier_mask;
#if defined(TARGET_SUPPORT_BARRIER_MASK_REG)
  barrier_mask = output().GetRegisterParameter(BARRIER_MASK);
#else
  barrier_mask = LoadFromOffset(
      output().thread(), compiler::target::Thread::write_barrier_mask_offset(),
      pointerType(output().repo().intPtr), true);
#endif
  barrier_mask =
      output().buildCast(LLVMTrunc, barrier_mask, output().repo().int8);
  return barrier_mask;
}

void AnonImpl::StoreToOffset(LValue base, int offset, LValue v) {
  StoreToOffset(base, output().constIntPtr(offset), v);
}

void AnonImpl::StoreToOffset(LValue base, LValue offset, LValue v) {
  LValue gep =
      output().buildGEPWithByteOffset(base, offset, pointerType(typeOf(v)));
  output().buildStore(v, gep);
}

void AnonImpl::StoreIntoObject(Instruction* instr,
                               LValue object,  // Object we are storing into.
                               LValue dest,    // Where we are storing into.
                               LValue value,   // Value we are storing.
                               compiler::Assembler::CanBeSmi can_value_be_smi) {
  LValue gep =
      BuildAccessPointer(object, dest, pointerType(output().tagged_type()));
  output().buildStore(value, gep);
  DiamondContinuationResolver diamond(*this, -1);
  diamond.BuildCmp([&]() {
    if (can_value_be_smi == compiler::Assembler::kValueCanBeSmi) {
      return IsSmi(value);
    }
    return output().repo().booleanFalse;
  });
  diamond.BuildLeft([&]() {
    // Smi case.
    return value;
  });
  diamond.BuildRight([&]() {
    // None Smi case
    LValue object_tag = LoadFieldFromOffset(
        object, compiler::target::Object::tags_offset(), output().repo().ref8);
    LValue value_tag = LoadFieldFromOffset(
        value, compiler::target::Object::tags_offset(), output().repo().ref8);
    LValue and_value = output().buildAnd(
        value_tag, output().buildShr(
                       object_tag,
                       output().constInt8(
                           compiler::target::RawObject::kBarrierOverlapShift)));
    LValue barrier_mask = GetBarrierMask();
    DiamondContinuationResolver resolver(*this, -1);
    resolver.RightHint();
    resolver.BuildCmp([&]() {
      LValue test_value = output().buildAnd(and_value, barrier_mask);
      return output().buildICmp(LLVMIntNE, test_value, output().constInt8(0));
    });
    resolver.BuildLeft([&]() {
      // should emit
      LValue entry = nullptr;
      const auto& stub = Code::ZoneHandle(object_store()->write_barrier_stub());
      std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
      callsite_info->set_needs_metadata(false);
      if (!stub.InVMIsolateHeap()) {
        callsite_info->set_type(CallSiteInfo::CallTargetType::kStubRelative);
        callsite_info->set_code(&stub);
        callsite_info->set_fpu_code(nullptr);
      } else {
        entry = LoadFromOffset(
            output().thread(),
            compiler::target::Thread::write_barrier_entry_point_offset(),
            pointerType(output().repo().ref8), true);
        callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
      }
      callsite_info->set_instr_size(kCallInstrSize);

      CallResolver::CallResolverParameter param(instr,
                                                std::move(callsite_info));
      CallResolver call_resolver(*this, -1, param);
      if (entry) call_resolver.SetCallTarget(entry);
      call_resolver.SetGParameter(
          Output::RegisterToParameterLoc(kWriteBarrierObjectReg), object);
      call_resolver.SetGParameter(
          Output::RegisterToParameterLoc(kWriteBarrierValueReg), value);
      call_resolver.set_callee_save_register(GetSharedStubCSR());
      call_resolver.BuildCall();
      return value;
    });
    resolver.BuildRight([&]() { return value; });
    return resolver.End();
  });
  diamond.End();
}

void AnonImpl::StoreIntoArray(Instruction* instr,
                              LValue object,  // Object we are storing into.
                              LValue dest,    // Where we are storing into.
                              LValue value,   // Value we are storing.
                              compiler::Assembler::CanBeSmi can_value_be_smi) {
  LValue gep =
      BuildAccessPointer(object, dest, pointerType(output().tagged_type()));
  output().buildStore(value, gep);
  DiamondContinuationResolver diamond(*this, -1);
  diamond.BuildCmp([&]() {
    if (can_value_be_smi == compiler::Assembler::kValueCanBeSmi) {
      return IsSmi(value);
    }
    return output().repo().booleanFalse;
  });
  diamond.BuildLeft([&]() {
    // Smi case.
    return value;
  });
  diamond.BuildRight([&]() {
    // None Smi case
    LValue object_tag = LoadFieldFromOffset(
        object, compiler::target::Object::tags_offset(), output().repo().ref8);
    LValue value_tag = LoadFieldFromOffset(
        value, compiler::target::Object::tags_offset(), output().repo().ref8);
    LValue and_value = output().buildAnd(
        value_tag, output().buildShr(
                       object_tag,
                       output().constInt8(
                           compiler::target::RawObject::kBarrierOverlapShift)));
    LValue barrier_mask = GetBarrierMask();
    DiamondContinuationResolver resolver(*this, -1);
    resolver.RightHint();
    resolver.BuildCmp([&]() {
      LValue test_value = output().buildAnd(and_value, barrier_mask);
      return output().buildICmp(LLVMIntNE, test_value, output().constInt8(0));
    });
    resolver.BuildLeft([&]() {
      // should emit
      std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
      callsite_info->set_needs_metadata(false);
      LValue entry = nullptr;
      const auto& stub =
          Code::ZoneHandle(object_store()->array_write_barrier_stub());
      if (!stub.InVMIsolateHeap()) {
        callsite_info->set_type(CallSiteInfo::CallTargetType::kStubRelative);
        callsite_info->set_code(&stub);
        callsite_info->set_fpu_code(nullptr);
      } else {
        entry = LoadFromOffset(
            output().thread(),
            compiler::target::Thread::array_write_barrier_entry_point_offset(),
            pointerType(output().repo().ref8), true);
        callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
      }
      callsite_info->set_instr_size(kCallInstrSize);
      CallResolver::CallResolverParameter param(instr,
                                                std::move(callsite_info));
      CallResolver call_resolver(*this, -1, param);
      if (entry) call_resolver.SetCallTarget(entry);
      call_resolver.SetGParameter(
          Output::RegisterToParameterLoc(kWriteBarrierObjectReg), object);
      call_resolver.SetGParameter(
          Output::RegisterToParameterLoc(kWriteBarrierValueReg), value);
      call_resolver.SetGParameter(
          Output::RegisterToParameterLoc(kWriteBarrierSlotReg), gep);
      call_resolver.set_callee_save_register(GetSharedStubCSR());
      call_resolver.BuildCall();

      return value;
    });
    resolver.BuildRight([&]() { return value; });
    return resolver.End();
  });
  diamond.End();
}

LValue AnonImpl::TryAllocate(AssemblerResolver& resolver,
                             Label& failure,
                             intptr_t cid,
                             intptr_t instance_size) {
  if (compiler::target::Heap::IsAllocatableInNewSpace(instance_size)) {
    LValue top_offset = output().buildGEPWithByteOffset(
        output().thread(), compiler::target::Thread::top_offset(),
        pointerType(output().repo().intPtr));
    LValue instance = output().buildLoad(top_offset);
    // TODO(koda): Protect against unsigned overflow here.
#if !defined(TARGET_ARCH_IS_64_BIT)
    LValue uadd_overflow =
        output().buildCall(output().repo().uaddWithOverflow32Intrinsic(),
                           instance, output().constInt32(instance_size));
#else
    LValue uadd_overflow =
        output().buildCall(output().repo().uaddWithOverflow64Intrinsic(),
                           instance, output().constInt64(instance_size));
#endif
    LValue end = LoadFromOffset(output().thread(),
                                compiler::target::Thread::end_offset(),
                                pointerType(output().repo().intPtr));

    LValue icmp = output().buildICmp(
        LLVMIntULE, end, output().buildExtractValue(uadd_overflow, 0));
    LValue overflowed = output().buildExtractValue(uadd_overflow, 1);
    LValue cond = output().buildOr(icmp, overflowed);
    resolver.BranchIf(ExpectFalse(cond), failure);
    output().buildStore(output().buildExtractValue(uadd_overflow, 0),
                        top_offset);
    EMASSERT(instance_size >= kHeapObjectTag);
    const uint32_t tags =
        compiler::target::MakeTagWordForNewSpaceObject(cid, instance_size);
    output().buildStore(
        output().constIntPtr(tags),
        BuildAccessPointer(
            instance,
            output().constIntPtr(compiler::target::Object::tags_offset()),
            pointerType(output().repo().intPtr)));
    instance =
        output().buildAdd(instance, output().constIntPtr(kHeapObjectTag));
    return WordToTagged(instance);
  } else {
    resolver.Branch(failure);
    return nullptr;
  }
}

LValue AnonImpl::BoxInteger32(Instruction* instr,
                              LValue value,
                              bool fit,
                              Representation rep) {
  EMASSERT(typeOf(value) == output().repo().int32);
#if defined(TARGET_ARCH_IS_32_BIT)
  LValue result = SmiTag(value);
  if (!fit) {
    AssemblerResolver resolver(*this);
    LValue fit_in_smi;
    if (kUnboxedInt32 == rep) {
      fit_in_smi = output().buildICmp(LLVMIntEQ, value, SmiUntag(result));
    } else {
      // high 2 bit must be 0 for 32bit arch unsigned integer.
      LValue and_value =
          output().buildAnd(value, output().constInt32(0xC0000000));
      fit_in_smi =
          output().buildICmp(LLVMIntEQ, and_value, output().constInt32(0));
    }
    Label slow_path("BoxInteger32SlowPath");
    resolver.BranchIfNot(ExpectTrue(fit_in_smi), slow_path);
    resolver.GotoMergeWithValue(result);

    resolver.Bind(slow_path);
    BoxAllocationSlowPath box_allocate(instr, mint_class(), *this);
    LValue mint = box_allocate.Allocate();
    StoreToOffset(mint, compiler::target::Mint::value_offset() - kHeapObjectTag,
                  value);
    resolver.GotoMergeWithValue(mint);
    result = resolver.End();
  }
#else
  LLVMOpcode opcode;
  if (kUnboxedInt32 == rep) {
    opcode = LLVMSExt;
  } else {
    opcode = LLVMZExt;
  }
  LValue result = output().buildCast(opcode, value, output().repo().int64);
  result = output().buildShl(result, output().constInt64(kSmiTagSize));
  result = WordToTagged(result);
#endif
  return result;
}

LBasicBlock AnonImpl::NewBasicBlock(const char* fmt, ...) {
  va_list va;
  va_start(va, fmt);
  char buf[256];
  vsnprintf(buf, 256, fmt, va);
  va_end(va);
  LBasicBlock bb = output().appendBasicBlock(buf);
  return bb;
}

void AnonImpl::SetCurrentBlockContinuation(LBasicBlock continuation) {
  current_bb_impl()->continuation = continuation;
}

LBasicBlock AnonImpl::GetCurrentBlockContinuation() {
  return current_bb_impl()->continuation;
}

ModifiedValuesMergeHelper::ModifiedValuesMergeHelper(AnonImpl& impl)
    : impl_(impl) {
  values_modified_ = impl.liveness().NewLiveBitVector();
}

ModifiedValuesMergeHelper::~ModifiedValuesMergeHelper() {
  // Check disabled.
  EMASSERT(impl().value_interceptor_ != this);
}

LValue ModifiedValuesMergeHelper::GetLLVMValue(int ssa_id) {
  auto found = current_modified_.find(ssa_id);
  if (found != current_modified_.end()) return found->second;
  return nullptr;
}

bool ModifiedValuesMergeHelper::SetLLVMValue(int ssa_id, LValue v) {
  current_modified_[ssa_id] = v;
  values_modified_->Add(ssa_id);
  return true;
}

void ModifiedValuesMergeHelper::AssignBlockInitialValues(LBasicBlock bb) {
  initial_modified_values_lookup_.emplace(bb, current_modified_);
}

void ModifiedValuesMergeHelper::InitialValuesInBlock(LBasicBlock bb) {
  auto found_initial_vals = initial_modified_values_lookup_.find(bb);
  EMASSERT(found_initial_vals != initial_modified_values_lookup_.end());
  current_modified_ = std::move(found_initial_vals->second);
}

void ModifiedValuesMergeHelper::InitialValuesInBlockToEmpty() {
  current_modified_.clear();
}

void ModifiedValuesMergeHelper::AssignModifiedValueToMerge(LBasicBlock bb) {
  modified_values_to_merge_.emplace(bb, current_modified_);
}

void ModifiedValuesMergeHelper::MergeModified() {
  EMASSERT(impl().value_interceptor_ != this);
  for (BitVector::Iterator it(values_modified_); !it.Done(); it.Advance()) {
    int need_to_merge = it.Current();
    LValue original = impl().GetLLVMValue(need_to_merge);
    LType type = typeOf(original);
    LValue phi = output().buildPhi(type);
    for (auto& pair : modified_values_to_merge_) {
      auto& value_lookup = pair.second;
      auto found = value_lookup.find(need_to_merge);
      if (found == value_lookup.end()) {
        addIncoming(phi, &original, &pair.first, 1);
      } else {
        addIncoming(phi, &found->second, &pair.first, 1);
      }
    }
    impl().SetLLVMValue(need_to_merge, phi);
  }
}

void ModifiedValuesMergeHelper::Enable() {
  old_value_interceptor_ = impl().value_interceptor_;
  impl().value_interceptor_ = this;
}

void ModifiedValuesMergeHelper::Disable() {
  impl().value_interceptor_ = old_value_interceptor_;
}

ContinuationResolver::ContinuationResolver(AnonImpl& impl, int ssa_id)
    : impl_(impl), ssa_id_(ssa_id) {}

DiamondContinuationResolver::DiamondContinuationResolver(AnonImpl& _impl,
                                                         int ssa_id)
    : ContinuationResolver(_impl, ssa_id),
      modified_value_helper_(_impl),
      hint_(0),
      building_block_index_(-1) {
  blocks_[0] = impl().NewBasicBlock("left_for_%d", ssa_id);
  blocks_[1] = impl().NewBasicBlock("right_for_%d", ssa_id);
  blocks_[2] = impl().NewBasicBlock("continuation_for_%d", ssa_id);
}

DiamondContinuationResolver& DiamondContinuationResolver::BuildCmp(
    std::function<LValue()> f) {
  LValue cmp_value = f();
  if (hint_ < 0) {
    cmp_value = impl().ExpectTrue(cmp_value);
  } else if (hint_ > 0) {
    cmp_value = impl().ExpectFalse(cmp_value);
  }
  output().buildCondBr(cmp_value, blocks_[0], blocks_[1]);
  modified_value_helper_.Enable();
  return *this;
}

DiamondContinuationResolver& DiamondContinuationResolver::BuildLeft(
    std::function<LValue()> f) {
  output().positionToBBEnd(blocks_[0]);
  LBasicBlock old_continuation = impl().GetCurrentBlockContinuation();
  building_block_index_ = 0;
  modified_value_helper_.InitialValuesInBlockToEmpty();
  values_[0] = f();
  output().buildBr(blocks_[2]);
  LBasicBlock from = blocks_[0];
  LBasicBlock current_continuation = impl().GetCurrentBlockContinuation();
  if (old_continuation != current_continuation) {
    from = current_continuation;
    blocks_[0] = from;
  }
  modified_value_helper_.AssignModifiedValueToMerge(from);
  return *this;
}

DiamondContinuationResolver& DiamondContinuationResolver::BuildRight(
    std::function<LValue()> f) {
  output().positionToBBEnd(blocks_[1]);
  LBasicBlock old_continuation = impl().GetCurrentBlockContinuation();
  building_block_index_ = 1;
  modified_value_helper_.InitialValuesInBlockToEmpty();
  values_[1] = f();
  output().buildBr(blocks_[2]);
  LBasicBlock from = blocks_[1];
  LBasicBlock current_continuation = impl().GetCurrentBlockContinuation();
  if (old_continuation != current_continuation) {
    from = current_continuation;
    blocks_[1] = from;
  }
  modified_value_helper_.AssignModifiedValueToMerge(from);
  return *this;
}

LValue DiamondContinuationResolver::End() {
  output().positionToBBEnd(blocks_[2]);
  modified_value_helper_.Disable();
  modified_value_helper_.MergeModified();
  LValue phi = output().buildPhi(typeOf(values_[0]));
  addIncoming(phi, values_, blocks_, 2);
  impl().SetCurrentBlockContinuation(blocks_[2]);
  return phi;
}

void DiamondContinuationResolver::BranchToOtherLeafOnCondition(LValue cond) {
  EMASSERT(building_block_index_ != -1);
  int other_leaf_index = building_block_index_ ^ 1;
  LBasicBlock diamond_continuation =
      impl().NewBasicBlock("diamond_continuation");

  cond = output().buildCall(output().repo().expectIntrinsic(), cond,
                            output().repo().booleanFalse);
  output().buildCondBr(cond, blocks_[other_leaf_index], diamond_continuation);
  output().positionToBBEnd(diamond_continuation);
  blocks_[building_block_index_] = diamond_continuation;
}

CallResolver::CallResolverParameter::CallResolverParameter(
    Instruction* instr,
    std::unique_ptr<CallSiteInfo> _callinfo)
    : call_instruction(instr), callsite_info(std::move(_callinfo)) {}

CallResolver::CallResolver(
    AnonImpl& impl,
    int ssa_id,
    CallResolver::CallResolverParameter& call_resolver_parameter)
    : ContinuationResolver(impl, ssa_id),
      call_resolver_parameter_(call_resolver_parameter),
      parameters_(kV8CCRegisterParameterCount,
                  LLVMGetUndef(output().tagged_type())) {
  // init parameters_.
  parameters_[Output::RegisterToParameterLoc(THR)] =
      LLVMGetUndef(output().repo().ref8);
}

void CallResolver::SetGParameter(int reg, LValue val) {
  EMASSERT(reg >= 0);
  if (reg < kV8CCRegisterParameterCount)
    parameters_[reg] = val;
  else
    EMASSERT(false && "invalid reg");
}

void CallResolver::AddStackParameter(LValue v) {
  if (typeOf(v) == output().repo().doubleType)
    v = impl().BitcastDoubleToInt64(v);
  parameters_.emplace_back(v);
  int which = parameter_current_slot_;
  LType type_of_v = typeOf(v);
  if (type_of_v == output().tagged_type()) {
    call_resolver_parameter_.callsite_info->MarkParameterBit(which, 1);
    parameter_current_slot_ += 1;
  } else if (type_of_v == output().repo().int64) {
#if defined(TARGET_ARCH_IS_32_BIT)
    call_resolver_parameter_.callsite_info->MarkParameterBit(which, 0);
    call_resolver_parameter_.callsite_info->MarkParameterBit(which + 1, 0);
    parameter_current_slot_ += 2;
#elif defined(TARGET_ARCH_IS_64_BIT)
    call_resolver_parameter_.callsite_info->MarkParameterBit(which, 0);
    parameter_current_slot_ += 1;
#else
#error unsupported arch
#endif
  } else {
    impl().set_exception_occured();
    THR_Print("FIXME: not supported type(maybe double)");
  }
  EMASSERT(parameters_.size() - kV8CCRegisterParameterCount <=
           ((call_resolver_parameter_.callsite_info->return_on_stack_pos() != -1
                 ? 1
                 : 0) +
            call_resolver_parameter_.callsite_info->stack_parameter_count()));
}

LValue CallResolver::GetStackParameter(size_t i) {
  i += kV8CCRegisterParameterCount;
  EMASSERT(parameters_.size() > i);
  return parameters_[i];
}

LValue CallResolver::BuildCall() {
  EMASSERT(call_resolver_parameter_.callsite_info->type() !=
           CallSiteInfo::CallTargetType::kUnspecify);
  EMASSERT(parameters_.size() - kV8CCRegisterParameterCount ==
           ((call_resolver_parameter_.callsite_info->return_on_stack_pos() != -1
                 ? 1
                 : 0) +
            call_resolver_parameter_.callsite_info->stack_parameter_count()));
  ExtractCallInfo();
  GenerateStatePointFunction();
  patchid_ = impl().NextPatchPoint();
  from_block_ = output().getInsertionBlock();
  if (!tail_call_)
    EmitCall();
  else
    EmitTailCall();

  EmitExceptionVars();
  // reset to continuation.
  if (need_invoke()) {
    EMASSERT(!tail_call_);
    output().positionToBBEnd(continuation_block_);
    impl().SetCurrentBlockContinuation(continuation_block_);
  }
  EmitRelocatesIfNeeded();
  EmitPatchPoint();
  EmitCSR();
  return call_value_;
}

void CallResolver::ExtractCallInfo() {
  // Current debug instr is the effecting instr.
  Instruction* live_out_instr = impl().CurrentDebugInstr();
  call_live_out_ = impl().liveness().GetCallOutAt(live_out_instr);
  if (impl().current_bb()->try_index() != kInvalidTryIndex &&
      call_resolver_parameter_.call_instruction->MayThrow()) {
    catch_block_ = impl().flow_graph()->graph_entry()->GetCatchEntry(
        impl().current_bb()->try_index());
  }
  if (call_resolver_parameter_.call_instruction->IsTailCall()) {
    tail_call_ = true;
  }
}

void CallResolver::GenerateStatePointFunction() {
  if (UNLIKELY(tail_call_)) {
    callee_type_ = output().repo().ref8;
    return;
  }
  return_type_ = impl().GetMachineRepresentationType(
      call_resolver_parameter_.call_instruction->representation());
  std::vector<LType> operand_value_types;
  for (LValue param : parameters_) {
    operand_value_types.emplace_back(typeOf(param));
  }
  LType callee_function_type =
      functionType(return_type_, operand_value_types.data(),
                   operand_value_types.size(), NotVariadic);
  callee_type_ = pointerType(callee_function_type);

  statepoint_function_ = output().getStatePointFunction(callee_type_);
}

void CallResolver::EmitCall() {
  std::vector<LValue> statepoint_operands;
  statepoint_operands.push_back(output().constInt64(patchid_));
  statepoint_operands.push_back(output().constInt32(
      call_resolver_parameter_.callsite_info->instr_size()));
  if (call_target_ == nullptr)
    statepoint_operands.push_back(LLVMGetUndef(callee_type_));
  else {
    LValue call_target = call_target_;
    if (typeOf(call_target) == output().tagged_type()) {
      call_target =
          output().buildCast(LLVMAddrSpaceCast, call_target, callee_type_);
    } else {
      call_target = output().buildBitCast(call_target, callee_type_);
    }

    statepoint_operands.push_back(call_target);
  }

  statepoint_operands.push_back(
      output().constInt32(parameters_.size()));           // # call params
  statepoint_operands.push_back(output().constInt32(0));  // flags
  for (LValue parameter : parameters_)
    statepoint_operands.push_back(parameter);
  statepoint_operands.push_back(output().constInt32(0));  // # transition args
  statepoint_operands.push_back(output().constInt32(0));  // # deopt arguments

  for (BitVector::Iterator it(call_live_out_); !it.Done(); it.Advance()) {
    int live_ssa_index = it.Current();
    if (impl().IsLazyValue(live_ssa_index)) continue;
    ValueDesc desc = impl().current_bb_impl()->GetValue(live_ssa_index);
    if (desc.type != ValueType::LLVMValue) continue;
    if (typeOf(desc.llvm_value) != output().tagged_type()) continue;
    if (live_ssa_index == impl().liveness().GetPPValueSSAIdx())
      pp_value_at_state_point_ = statepoint_operands.size();
    gc_desc_map_.emplace(live_ssa_index, statepoint_operands.size());
    statepoint_operands.emplace_back(desc.llvm_value);
  }
  while (need_invoke()) {
    IRTranslatorBlockImpl* block_impl =
        impl().GetTranslatorBlockImpl(catch_block_);
    auto& exception_params = block_impl->exception_params;
    if (exception_params.empty()) break;
    if (!block_impl->request_exception_env_) break;
    Environment* env = impl().pending_deoptimization_env_;
    EMASSERT(env != nullptr);
    env = env->Outermost();
    EMASSERT(env != nullptr);
    FlowGraph* flow_graph = impl().flow_graph();
    for (intptr_t i = 0; i < flow_graph->variable_count(); ++i) {
      auto found = exception_params.find(i);
      if (found == exception_params.end()) continue;
      Value* use = env->ValueAt(i);
      Definition* def = use->definition();
      LValue val;
      if (def->representation() == kTagged) {
        val = impl().GetLLVMValue(use);
      } else {
        // We can't handle this function anymore.
        val = LLVMGetUndef(output().tagged_type());
        impl().set_exception_occured();
      }
      // add the exception_gc_desc_map_
      exception_gc_desc_map_.emplace(i, statepoint_operands.size());
      statepoint_operands.emplace_back(val);
    }
    break;
  }

  if (!need_invoke()) {
    statepoint_value_ =
        output().buildCall(statepoint_function_, statepoint_operands.data(),
                           statepoint_operands.size());
  } else {
    continuation_block_ =
        impl().NewBasicBlock("continuation_block_for_%d", ssa_id());
    char buf[256];
    snprintf(buf, 256, "exception_block_from_%d",
             static_cast<int>(catch_block_->block_id()));

    exception_native_bb_ = output().appendBasicBlock(buf);
    statepoint_value_ = output().buildInvoke(
        statepoint_function_, statepoint_operands.data(),
        statepoint_operands.size(), continuation_block_, exception_native_bb_);
  }
  LLVMSetInstructionCallConv(statepoint_value_, LLVMV8CallConv);
}

void CallResolver::EmitTailCall() {
  std::vector<LValue> patchpoint_operands;
  patchpoint_operands.push_back(output().constInt64(patchid_));
  patchpoint_operands.push_back(output().constInt32(
      call_resolver_parameter_.callsite_info->instr_size()));
  if (call_target_ == nullptr)
    patchpoint_operands.push_back(LLVMGetUndef(callee_type_));
  else {
    LValue call_target = call_target_;
    if (typeOf(call_target) == output().tagged_type()) {
      call_target =
          output().buildCast(LLVMAddrSpaceCast, call_target, callee_type_);
    } else {
      call_target = output().buildBitCast(call_target, callee_type_);
    }

    patchpoint_operands.push_back(call_target);
  }
  patchpoint_operands.push_back(
      output().constInt32(parameters_.size()));  // # call params
  for (LValue parameter : parameters_)
    patchpoint_operands.push_back(parameter);
  LValue patchpoint_ret = output().buildCall(
      output().repo().patchpointVoidIntrinsic(), patchpoint_operands.data(),
      patchpoint_operands.size());
  LLVMSetInstructionCallConv(patchpoint_ret, LLVMV8CallConv);
  output().buildReturnForTailCall();
}

void CallResolver::EmitRelocatesIfNeeded() {
  if (tail_call_) return;
  for (auto& gc_relocate : gc_desc_map_) {
    LValue relocated = output().buildCall(
        output().repo().gcRelocateIntrinsic(), statepoint_value_,
        output().constInt32(gc_relocate.second),
        output().constInt32(gc_relocate.second));
    impl().SetLLVMValue(gc_relocate.first, relocated);
  }

  LValue intrinsic = output().getGCResultFunction(return_type_);
  call_value_ = output().buildCall(intrinsic, statepoint_value_);
}

void CallResolver::EmitPatchPoint() {
  std::unique_ptr<CallSiteInfo> callsite_info(
      std::move(call_resolver_parameter_.callsite_info));
  callsite_info->set_is_tailcall(tail_call_);
  callsite_info->set_patchid(patchid_);
  if (need_invoke())
    callsite_info->set_try_index(impl().current_bb()->try_index());
  impl().SubmitStackMap(std::move(callsite_info));
}

void CallResolver::EmitExceptionVars() {
  // Emit exception vars.
  if (need_invoke()) {
    output().positionToBBEnd(exception_native_bb_);

    LValue landing_pad = output().buildLandingPad();
    LValue exception_val =
        output().buildCall(output().repo().gcExceptionIntrinsic(), landing_pad);
    LValue stacktrace_val = output().buildCall(
        output().repo().gcExceptionDataIntrinsic(), landing_pad);
    IRTranslatorBlockImpl* block_impl =
        impl().GetTranslatorBlockImpl(catch_block_);
    auto& exception_params = block_impl->exception_params;
    addIncoming(block_impl->stacktrace_val, &stacktrace_val,
                &exception_native_bb_, 1);
    addIncoming(block_impl->exception_val, &exception_val,
                &exception_native_bb_, 1);
    for (auto& p : exception_gc_desc_map_) {
      auto found = exception_params.find(p.first);
      EMASSERT(found != exception_params.end());
      LValue relocated = output().buildCall(
          output().repo().gcRelocateIntrinsic(), landing_pad,
          output().constInt32(p.second), output().constInt32(p.second));
      addIncoming(found->second, &relocated, &exception_native_bb_, 1);
    }
    // PP Value
    EMASSERT(pp_value_at_state_point_ != 0);
    LValue new_pp =
        output().buildCall(output().repo().gcRelocateIntrinsic(), landing_pad,
                           output().constInt32(pp_value_at_state_point_),
                           output().constInt32(pp_value_at_state_point_));
    addIncoming(block_impl->exception_pp_val, &new_pp, &exception_native_bb_,
                1);
    output().buildBr(block_impl->native_bb);
  }
}

void CallResolver::EmitCSR() {
  if (LIKELY(!callee_save_registers_)) return;
  output().assignCSRAttr(statepoint_value_, callee_save_registers_);
}

BoxAllocationSlowPath::BoxAllocationSlowPath(Instruction* instruction,
                                             const Class& cls,
                                             AnonImpl& impl)
    : instruction_(instruction), cls_(cls), impl_(impl) {}

LValue BoxAllocationSlowPath::Allocate() {
  const intptr_t instance_size = compiler::target::Class::GetInstanceSize(cls_);
  const classid_t cid = compiler::target::Class::GetId(cls_);
  AssemblerResolver resolver(impl());
  Label slow_path("BoxAllocationSlowPath:slow_path");
  LValue instance = impl().TryAllocate(resolver, slow_path, cid, instance_size);
  if (instance) resolver.GotoMergeWithValue(instance);
  resolver.Bind(slow_path);
  resolver.GotoMergeWithValue(BuildSlowPath());
  return resolver.End();
}

LValue BoxAllocationSlowPath::BuildSlowPath() {
  const Code& stub = Code::ZoneHandle(
      impl().zone(), StubCode::GetAllocationStubForClass(cls_));
  return impl().GenerateCall(instruction_, instruction_->token_pos(), -1, stub,
                             RawPcDescriptors::kOther, 0, {});
}

ComparisonResolver::ComparisonResolver(AnonImpl& impl)
    : FlowGraphVisitor(impl.flow_graph()->reverse_postorder()),
      impl_(impl),
      result_(nullptr) {}

void ComparisonResolver::VisitStrictCompare(StrictCompareInstr* instr) {
  LValue left = impl().GetLLVMValue(instr->InputAt(0));
  LValue right = impl().GetLLVMValue(instr->InputAt(1));
  if (instr->needs_number_check()) {
    Label reference_compare("reference_compare_%d", instr->ssa_temp_index()),
        check_mint("check_mint_%d", instr->ssa_temp_index());
    AssemblerResolver resolver(impl());
    LValue left_is_smi = impl().IsSmi(left);
    resolver.BranchIf(left_is_smi, reference_compare);
    LValue right_is_smi = impl().IsSmi(left);
    resolver.BranchIf(right_is_smi, reference_compare);
    LValue left_is_double = impl().CompareClassId(left, kDoubleCid);
    resolver.BranchIfNot(left_is_double, check_mint);
    LValue right_is_double = impl().CompareClassId(right, kDoubleCid);
    resolver.GotoMergeWithValueIfNot(right_is_double,
                                     output().repo().booleanFalse);
    LValue left_double_bytes = impl().LoadFieldFromOffset(
        left, compiler::target::Double::value_offset(), output().repo().ref64);
    LValue right_double_bytes = impl().LoadFieldFromOffset(
        right, compiler::target::Double::value_offset(), output().repo().ref64);
    resolver.GotoMergeWithValue(
        output().buildICmp(LLVMIntEQ, left_double_bytes, right_double_bytes));

    resolver.Bind(check_mint);
    LValue left_is_mint = impl().CompareClassId(left, kMintCid);
    resolver.BranchIfNot(left_is_mint, reference_compare);
    LValue right_is_mint = impl().CompareClassId(right, kMintCid);
    resolver.GotoMergeWithValueIfNot(right_is_mint,
                                     output().repo().booleanFalse);
    LValue left_mint_val = impl().LoadFieldFromOffset(
        left, compiler::target::Mint::value_offset(), output().repo().ref64);
    LValue right_mint_val = impl().LoadFieldFromOffset(
        right, compiler::target::Mint::value_offset(), output().repo().ref64);
    resolver.GotoMergeWithValue(
        output().buildICmp(LLVMIntEQ, left_mint_val, right_mint_val));

    resolver.Bind(reference_compare);
    LValue left_word = impl().TaggedToWord(left);
    LValue right_word = impl().TaggedToWord(right);
    resolver.GotoMergeWithValue(
        output().buildICmp(LLVMIntEQ, left_word, right_word));

    result_ = resolver.End();
  } else {
    result_ = output().buildICmp(
        instr->kind() == Token::kEQ_STRICT ? LLVMIntEQ : LLVMIntNE, left,
        right);
  }
}

ReturnRepresentationDeducer::ReturnRepresentationDeducer(AnonImpl& impl)
    : FlowGraphVisitor(impl.flow_graph()->reverse_postorder()),
      rep_(kNoRepresentation) {}

void ReturnRepresentationDeducer::VisitReturn(ReturnInstr* instr) {
  Definition* def = instr->value()->definition();
  rep_ = def->representation();
}

Representation ReturnRepresentationDeducer::Deduce() {
  VisitBlocks();
  return rep_;
}

static LLVMIntPredicate TokenKindToSmiCondition(Token::Kind kind) {
  switch (kind) {
    case Token::kEQ:
      return LLVMIntEQ;
    case Token::kNE:
      return LLVMIntNE;
    case Token::kLT:
      return LLVMIntSLT;
    case Token::kGT:
      return LLVMIntSGT;
    case Token::kLTE:
      return LLVMIntSLE;
    case Token::kGTE:
      return LLVMIntSGE;
    default:
      UNREACHABLE();
  }
}

static LLVMRealPredicate TokenKindToDoubleCondition(Token::Kind kind) {
  switch (kind) {
    case Token::kEQ:
      return LLVMRealOEQ;
    case Token::kNE:
      return LLVMRealONE;
    case Token::kLT:
      return LLVMRealOLT;
    case Token::kGT:
      return LLVMRealOGT;
    case Token::kLTE:
      return LLVMRealOLE;
    case Token::kGTE:
      return LLVMRealOGE;
    default:
      UNREACHABLE();
  }
}

void ComparisonResolver::VisitEqualityCompare(EqualityCompareInstr* instr) {
  LValue cmp_value;
  LValue left = impl().GetLLVMValue(instr->InputAt(0));
  LValue right = impl().GetLLVMValue(instr->InputAt(1));
  if (instr->operation_cid() == kSmiCid) {
    EMASSERT(typeOf(left) == output().tagged_type());
    EMASSERT(typeOf(right) == output().tagged_type());
    cmp_value =
        output().buildICmp(TokenKindToSmiCondition(instr->kind()),
                           impl().SmiUntag(left), impl().SmiUntag(right));
  } else if (instr->operation_cid() == kMintCid) {
    EMASSERT(typeOf(left) == output().repo().int64);
    EMASSERT(typeOf(right) == output().repo().int64);
    cmp_value =
        output().buildICmp(TokenKindToSmiCondition(instr->kind()), left, right);
  } else {
    EMASSERT(instr->operation_cid() == kDoubleCid);
    EMASSERT(typeOf(left) == output().repo().doubleType);
    EMASSERT(typeOf(right) == output().repo().doubleType);
    cmp_value = output().buildFCmp(TokenKindToDoubleCondition(instr->kind()),
                                   left, right);
  }
  result_ = cmp_value;
}

void ComparisonResolver::VisitCheckedSmiComparison(
    CheckedSmiComparisonInstr* instr) {
  LValue left = impl().GetLLVMValue(instr->InputAt(0));
  LValue right = impl().GetLLVMValue(instr->InputAt(1));

  intptr_t left_cid = instr->left()->Type()->ToCid();
  intptr_t right_cid = instr->right()->Type()->ToCid();
  DiamondContinuationResolver diamond(impl(), instr->ssa_temp_index());
  diamond.LeftHint();
  diamond.BuildCmp([&] {
    LValue is_smi;
    if (instr->left()->definition() == instr->right()->definition()) {
      is_smi = impl().IsSmi(left);
    } else if (left_cid == kSmiCid) {
      is_smi = impl().IsSmi(right);
    } else if (right_cid == kSmiCid) {
      is_smi = impl().IsSmi(left);
    } else {
      LValue left_address = impl().TaggedToWord(left);
      LValue right_address = impl().TaggedToWord(right);
      LValue orr_address = output().buildOr(left_address, right_address);
      LValue and_value =
          output().buildAnd(orr_address, output().constIntPtr(kSmiTagMask));
      is_smi = output().buildICmp(LLVMIntNE, and_value,
                                  output().constIntPtr(kSmiTagMask));
    }
    return is_smi;
  });
  diamond.BuildLeft([&] {
    // both are smi
    return output().buildICmp(TokenKindToSmiCondition(instr->kind()),
                              impl().SmiUntag(left), impl().SmiUntag(right));
  });
  diamond.BuildRight([&] {
    impl().PushArgument(left);
    impl().PushArgument(right);
    const auto& selector = String::Handle(instr->call()->Selector());
    const auto& arguments_descriptor =
        Array::Handle(ArgumentsDescriptor::NewBoxed(/*type_args_len=*/0,
                                                    /*num_arguments=*/2));
    LValue boolean_object = impl().GenerateMegamorphicInstanceCall(
        instr, selector, arguments_descriptor, instr->call()->deopt_id(),
        instr->token_pos());
    LValue true_object = impl().LoadObject(Bool::True());
    LValue cmp = output().buildICmp(LLVMIntEQ, boolean_object, true_object);
    if (instr->is_negated()) cmp = output().buildNot(cmp);
    return cmp;
  });
  result_ = diamond.End();
}

void ComparisonResolver::VisitCaseInsensitiveCompare(
    CaseInsensitiveCompareInstr* instr) {
  UNREACHABLE();
}

void ComparisonResolver::VisitRelationalOp(RelationalOpInstr* instr) {
  LValue cmp_value;
  LValue left = impl().GetLLVMValue(instr->InputAt(0));
  LValue right = impl().GetLLVMValue(instr->InputAt(1));
  if (instr->operation_cid() == kSmiCid) {
    EMASSERT(typeOf(left) == output().tagged_type());
    EMASSERT(typeOf(right) == output().tagged_type());
    cmp_value =
        output().buildICmp(TokenKindToSmiCondition(instr->kind()),
                           impl().SmiUntag(left), impl().SmiUntag(right));
  } else if (instr->operation_cid() == kMintCid) {
    EMASSERT(typeOf(left) == output().repo().int64);
    EMASSERT(typeOf(right) == output().repo().int64);
    cmp_value =
        output().buildICmp(TokenKindToSmiCondition(instr->kind()), left, right);
  } else {
    EMASSERT(instr->operation_cid() == kDoubleCid);
    EMASSERT(typeOf(left) == output().repo().doubleType);
    EMASSERT(typeOf(right) == output().repo().doubleType);
    cmp_value = output().buildFCmp(TokenKindToDoubleCondition(instr->kind()),
                                   left, right);
  }
  result_ = cmp_value;
}

void ComparisonResolver::VisitDoubleTestOp(DoubleTestOpInstr* instr) {
  LValue value = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(value) == output().repo().doubleType);
  LValue cmp_value;
  if (instr->op_kind() == MethodRecognizer::kDouble_getIsNaN) {
    cmp_value = output().buildFCmp(LLVMRealUNO, value, value);
  } else {
    ASSERT(instr->op_kind() == MethodRecognizer::kDouble_getIsInfinite);
    value = output().buildCall(output().repo().doubleAbsIntrinsic(), value);
    cmp_value =
        output().buildFCmp(LLVMRealOEQ, value, output().constDouble(INFINITY));
  }
  const bool is_negated = instr->kind() != Token::kEQ;
  if (is_negated) {
    cmp_value =
        output().buildICmp(LLVMIntEQ, cmp_value, output().repo().booleanFalse);
  }
  result_ = cmp_value;
}

Label::Label(const char* fmt, ...) : bb_(nullptr) {
  va_list va;
  va_start(va, fmt);
  char buf[256];
  vsnprintf(buf, 256, fmt, va);
  va_end(va);
  name_.assign(buf);
}

void Label::InitBBIfNeeded(AnonImpl& impl) {
  if (bb_) return;
  bb_ = impl.NewBasicBlock(name_.c_str());
}

AssemblerResolver::AssemblerResolver(AnonImpl& impl)
    : ModifiedValuesMergeHelper(impl) {
  merge_ = impl.NewBasicBlock("AssemblerResolver::Merge");
  Enable();
}

AssemblerResolver::~AssemblerResolver() {}

void AssemblerResolver::Branch(Label& label) {
  label.InitBBIfNeeded(impl());
  output().buildBr(label.basic_block());
  AssignBlockInitialValues(label.basic_block());
}

void AssemblerResolver::BranchIf(LValue cond, Label& label) {
  LBasicBlock continuation =
      impl().NewBasicBlock("AssemblerResolver::BranchIf");
  label.InitBBIfNeeded(impl());
  output().buildCondBr(cond, label.basic_block(), continuation);
  AssignBlockInitialValues(label.basic_block());
  output().positionToBBEnd(continuation);
}

void AssemblerResolver::BranchIfNot(LValue cond, Label& label) {
  LBasicBlock continuation =
      impl().NewBasicBlock("AssemblerResolver::BranchIfNot");
  label.InitBBIfNeeded(impl());
  output().buildCondBr(cond, continuation, label.basic_block());
  AssignBlockInitialValues(label.basic_block());
  output().positionToBBEnd(continuation);
}

void AssemblerResolver::Bind(Label& label) {
  EMASSERT(
      (LLVMGetBasicBlockTerminator(output().getInsertionBlock()) != nullptr) &&
      "current bb has not yet been terminated");
  label.InitBBIfNeeded(impl());
  LBasicBlock current = label.basic_block();
  output().positionToBBEnd(current);
  InitialValuesInBlock(current);
}

LValue AssemblerResolver::End() {
  EMASSERT(
      (LLVMGetBasicBlockTerminator(output().getInsertionBlock()) != nullptr) &&
      "current bb has not yet been terminated");
  output().positionToBBEnd(merge_);
  impl().SetCurrentBlockContinuation(merge_);

  Disable();
  MergeModified();
  if (merge_values_.empty()) return nullptr;
  EMASSERT(merge_values_.size() == merge_blocks_.size());
  LType type = typeOf(merge_values_.front());
  LValue phi = output().buildPhi(type);
  for (LValue v : merge_values_) {
    EMASSERT(type == typeOf(v));
  }
  addIncoming(phi, merge_values_.data(), merge_blocks_.data(),
              merge_values_.size());
  return phi;
}

void AssemblerResolver::GotoMerge() {
  LBasicBlock current = output().getInsertionBlock();
  merge_blocks_.emplace_back(current);
  AssignModifiedValueToMerge(current);
  output().buildBr(merge_);
}

void AssemblerResolver::GotoMergeIf(LValue cond) {
  LBasicBlock current = output().getInsertionBlock();
  merge_blocks_.emplace_back(current);
  AssignModifiedValueToMerge(current);
  LBasicBlock continuation =
      impl().NewBasicBlock("AssemblerResolver::GotoMergeIf");
  output().buildCondBr(cond, merge_, continuation);
  output().positionToBBEnd(continuation);
}

void AssemblerResolver::GotoMergeIfNot(LValue cond) {
  LBasicBlock current = output().getInsertionBlock();
  merge_blocks_.emplace_back(current);
  AssignModifiedValueToMerge(current);
  LBasicBlock continuation =
      impl().NewBasicBlock("AssemblerResolver::GotoMergeIf");
  output().buildCondBr(cond, continuation, merge_);
  output().positionToBBEnd(continuation);
}

void AssemblerResolver::GotoMergeWithValue(LValue v) {
  merge_values_.emplace_back(v);
  GotoMerge();
}

void AssemblerResolver::GotoMergeWithValueIf(LValue cond, LValue v) {
  LBasicBlock current = output().getInsertionBlock();
  LBasicBlock continuation =
      impl().NewBasicBlock("AssemblerResolver::GotoMergeWithValueIf");
  merge_values_.emplace_back(v);
  merge_blocks_.emplace_back(current);
  AssignModifiedValueToMerge(current);
  output().buildCondBr(cond, merge_, continuation);
  output().positionToBBEnd(continuation);
}

void AssemblerResolver::GotoMergeWithValueIfNot(LValue cond, LValue v) {
  LBasicBlock continuation =
      impl().NewBasicBlock("AssemblerResolver::GotoMergeWithValueIfNot");
  merge_values_.emplace_back(v);
  LBasicBlock current = output().getInsertionBlock();
  merge_blocks_.emplace_back(current);
  AssignModifiedValueToMerge(current);
  output().buildCondBr(cond, continuation, merge_);
  output().positionToBBEnd(continuation);
}

BlockScope::BlockScope(AnonImpl& impl, BlockEntryInstr* bb)
    : saved_(impl.current_bb()), impl_(impl) {
  impl.current_bb_ = bb;
  impl.current_bb_impl_ = impl.GetTranslatorBlockImpl(bb);
}

BlockScope::~BlockScope() {
  impl_.current_bb_ = saved_;
  impl_.current_bb_impl_ = impl_.GetTranslatorBlockImpl(saved_);
}
#if defined(TARGET_ARCH_ARM)
#include "vm/compiler/backend/llvm/ir_translator_arm.cc"
#elif defined(TARGET_ARCH_ARM64)
#include "vm/compiler/backend/llvm/ir_translator_arm64.cc"
#else
#error unsupported arch
#endif
}  // namespace

struct IRTranslator::Impl : public AnonImpl {};

IRTranslator::IRTranslator(FlowGraph* flow_graph, Precompiler* precompiler)
    : FlowGraphVisitor(flow_graph->reverse_postorder()) {
  // dynamic function has multiple entry points
  // which LLVM does not suppport.
  InitLLVM();
  impl_.reset(new Impl);
  impl().flow_graph_ = flow_graph;
  impl().precompiler_ = precompiler;
  impl().thread_ = Thread::Current();
  impl().compiler_state_.reset(new CompilerState(
      String::Handle(flow_graph->zone(),
                     flow_graph->parsed_function().function().UserVisibleName())
          .ToCString()));
  impl().liveness_analysis_.reset(new LivenessAnalysis(flow_graph));
  impl().output_.reset(new Output(impl().compiler_state()));
  // init parameter desc
  RegisterParameterDesc parameter_desc;
  LType tagged_type = output().repo().tagged_type;
  LType int64_type = output().repo().int64;
  GraphEntryInstr* graph_entry = flow_graph->graph_entry();
  FunctionEntryInstr* function_entry =
      graph_entry->SuccessorAt(0)->AsFunctionEntry();
  EMASSERT(function_entry != nullptr);

  std::vector<ParameterInstr*> params;
  for (Definition* def : *function_entry->initial_definitions()) {
    ParameterInstr* param = def->AsParameter();
    if (param) params.emplace_back(param);
  }
  EMASSERT(params.size() ==
           static_cast<size_t>(flow_graph->num_direct_parameters()));
  std::sort(params.begin(), params.end(),
            [](const ParameterInstr* lhs, const ParameterInstr* rhs) {
              return lhs->index() < rhs->index();
            });
  int parameter_end = params.size();
  for (ParameterInstr* param : params) {
    parameter_end -= 1;
    switch (param->representation()) {
      case kUnboxedDouble:
      case kUnboxedInt64:
        parameter_desc.emplace_back(-parameter_end - 1, int64_type);
        break;
      case kTagged:
        parameter_desc.emplace_back(-parameter_end - 1, tagged_type);
        break;
      default:
        impl().output_.reset();
        impl().compiler_state_.reset();
        THR_Print("function %s will not compile for param: %s\n",
                  flow_graph->parsed_function().function().ToCString(),
                  param->ToCString());
    }
  }
  EMASSERT(parameter_end == 0);

  // init output().
  output().initializeBuild(parameter_desc, impl().DeduceReturnType());
  // initialize exception vars.
  impl().CollectExceptionVars();
}

IRTranslator::~IRTranslator() {}

void IRTranslator::Translate() {
#if 1
  if (!impl().output_) return;
  if (!impl().liveness().Analyze()) return;
  VisitBlocks();
  if (impl().exception_occured_) return;
  impl().End();
#elif !defined(PRODUCT)
  // FlowGraphPrinter::PrintGraph("IRTranslator", impl().flow_graph_);
#endif
}

Output& IRTranslator::output() {
  return impl().output();
}

void IRTranslator::VisitBlockEntry(BlockEntryInstr* block) {
  impl().StartTranslate(block);
  IRTranslatorBlockImpl* block_impl = impl().current_bb_impl();
  block_impl->try_index = block->try_index();
  if (!block_impl->exception_block) impl().MergePredecessors(block);
}

void IRTranslator::VisitBlockEntryWithInitialDefs(
    BlockEntryWithInitialDefs* block) {
  VisitBlockEntry(block);
  for (Definition* def : *block->initial_definitions()) {
    def->Accept(this);
  }
}

void IRTranslator::VisitGraphEntry(GraphEntryInstr* instr) {
  VisitBlockEntryWithInitialDefs(instr);
  impl().SetLLVMValue(impl().liveness().GetPPValueSSAIdx(), output().pp());
}

void IRTranslator::VisitJoinEntry(JoinEntryInstr* instr) {
  VisitBlockEntry(instr);
  if (instr->phis()) {
    for (PhiInstr* phi : *instr->phis()) {
      VisitPhi(phi);
    }
  }
}

void IRTranslator::VisitTargetEntry(TargetEntryInstr* instr) {
  VisitBlockEntry(instr);
}

void IRTranslator::VisitFunctionEntry(FunctionEntryInstr* instr) {
  EMASSERT(!impl().visited_function_entry_);
  impl().visited_function_entry_ = true;
  LBasicBlock bb = impl().EnsureNativeBB(instr);
  output().buildBr(bb);
  VisitBlockEntryWithInitialDefs(instr);
}

void IRTranslator::VisitNativeEntry(NativeEntryInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitOsrEntry(OsrEntryInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitIndirectEntry(IndirectEntryInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitCatchBlockEntry(CatchBlockEntryInstr* instr) {
  IRTranslatorBlockImpl* block_impl = impl().GetCatchBlockImpl(instr);
  output().positionToBBEnd(block_impl->native_bb);
  VisitBlockEntryWithInitialDefs(instr);
}

void IRTranslator::VisitPhi(PhiInstr* instr) {
  impl().SetCurrentInstr(instr);
  LType phi_type = impl().GetMachineRepresentationType(instr->representation());

  LValue phi = output().buildPhi(phi_type);
  bool should_add_to_tf_phi_worklist = false;
  intptr_t predecessor_count = impl().current_bb_->PredecessorCount();
  IRTranslatorBlockImpl* block_impl = impl().current_bb_impl();

  for (intptr_t i = 0; i < predecessor_count; ++i) {
    BlockEntryInstr* pred = impl().current_bb()->PredecessorAt(i);
    int ssa_value = instr->InputAt(i)->definition()->ssa_temp_index();
    if (impl().IsBBStartedToTranslate(pred)) {
      LValue value =
          impl().EnsurePhiInputAndPosition(pred, ssa_value, phi_type);
      LBasicBlock llvmbb = impl().GetNativeBBContinuation(pred);
      addIncoming(phi, &value, &llvmbb, 1);
    } else {
      should_add_to_tf_phi_worklist = true;
      block_impl->not_merged_phis.emplace_back();
      auto& not_merged_phi = block_impl->not_merged_phis.back();
      not_merged_phi.phi = phi;
      not_merged_phi.pred = pred;
      not_merged_phi.value = ssa_value;
    }
  }
  if (should_add_to_tf_phi_worklist)
    impl().phi_rebuild_worklist_.push_back(impl().current_bb_);
  block_impl->SetLLVMValue(instr->ssa_temp_index(), phi);
  if (instr->HasPairRepresentation())
    block_impl->SetLLVMValue(instr->ssa_temp_index() + 1,
                             LLVMGetUndef(output().repo().boolean));
}

void IRTranslator::VisitRedefinition(RedefinitionInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitParameter(ParameterInstr* instr) {
  int param_index = instr->index();
  LValue result;
  IRTranslatorBlockImpl* block_impl = impl().current_bb_impl();
  if (param_index < impl().flow_graph()->num_direct_parameters()) {
    result = output().parameter(param_index);
    if (instr->representation() == kUnboxedDouble)
      result = impl().BitcastInt64ToDouble(result);
  } else {
    EMASSERT(block_impl->exception_block);
    auto found = block_impl->exception_params.find(param_index);
    EMASSERT(found != block_impl->exception_params.end());
    result = found->second;
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitNativeParameter(NativeParameterInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitLoadIndexedUnsafe(LoadIndexedUnsafeInstr* instr) {
  EMASSERT(instr->RequiredInputRepresentation(0) == kTagged);  // It is a Smi.
  impl().SetCurrentInstr(instr);
  LValue index_smi = impl().TaggedToWord(impl().GetLLVMValue(instr->index()));
  EMASSERT(instr->base_reg() == FP);
  LValue offset = output().buildShl(
      index_smi,
      output().constIntPtr(compiler::target::kWordSizeLog2 - kSmiTagSize));
  offset = output().buildAdd(offset, output().constIntPtr(instr->offset()));
  LValue access =
      impl().BuildAccessPointer(output().fp(), offset, instr->representation());
  LValue value = output().buildLoad(access);
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitStoreIndexedUnsafe(StoreIndexedUnsafeInstr* instr) {
  ASSERT(instr->RequiredInputRepresentation(
             StoreIndexedUnsafeInstr::kIndexPos) == kTagged);  // It is a Smi.
  impl().SetCurrentInstr(instr);
  LValue index_smi = impl().GetLLVMValue(instr->index());
  EMASSERT(instr->base_reg() == FP);
  LValue offset = output().buildShl(
      index_smi,
      output().constInt32(compiler::target::kWordSizeLog2 - kSmiTagSize));
  offset = output().buildAdd(offset, output().constIntPtr(instr->offset()));
  LValue gep =
      impl().BuildAccessPointer(output().fp(), offset, instr->representation());

  LValue value = impl().GetLLVMValue(instr->value());
  output().buildStore(value, gep);
}

void IRTranslator::VisitTailCall(TailCallInstr* instr) {
  impl().SetCurrentInstr(instr);
  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  LValue code_object = impl().LoadObject(instr->code());
  LValue entry_gep = output().buildGEPWithByteOffset(
      code_object,
      output().constIntPtr(compiler::target::Code::entry_point_offset() -
                           kHeapObjectTag),
      pointerType(output().repo().ref8));
  LValue entry = output().buildLoad(entry_gep);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
  callsite_info->set_instr_size(kCallInstrSize);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(impl(), -1, param);
  resolver.SetGParameter(Output::RegisterToParameterLoc(CODE_REG), code_object);
  resolver.SetCallTarget(entry);
  // add register parameter.
  resolver.SetGParameter(Output::RegisterToParameterLoc(ARGS_DESC_REG),
                         output().args_desc());
  resolver.BuildCall();
}

void IRTranslator::VisitParallelMove(ParallelMoveInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitPushArgument(PushArgumentInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  impl().PushArgument(value);
}

void IRTranslator::VisitReturn(ReturnInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  output().buildRet(value);
}

void IRTranslator::VisitNativeReturn(NativeReturnInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitThrow(ThrowInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue exception = impl().GetLLVMValue(instr->exception());
  impl().PushArgument(exception);
  impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                             kThrowRuntimeEntry, 1, false);
  output().buildCall(output().repo().trapIntrinsic());
  output().buildRetUndef();
}

void IRTranslator::VisitReThrow(ReThrowInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue exception = impl().GetLLVMValue(instr->exception());
  impl().PushArgument(exception);
  LValue stack_trace = impl().GetLLVMValue(instr->stacktrace());
  impl().PushArgument(stack_trace);
  impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                             kReThrowRuntimeEntry, 2, false);
  output().buildCall(output().repo().trapIntrinsic());
  output().buildRetUndef();
}

void IRTranslator::VisitStop(StopInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitGoto(GotoInstr* instr) {
  impl().SetCurrentInstr(instr);
  JoinEntryInstr* successor = instr->successor();
  LBasicBlock bb = impl().EnsureNativeBB(successor);
  output().buildBr(bb);
  impl().EndCurrentBlock();
}

void IRTranslator::VisitIndirectGoto(IndirectGotoInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitBranch(BranchInstr* instr) {
  impl().SetCurrentInstr(instr);
  TargetEntryInstr* true_successor = instr->true_successor();
  TargetEntryInstr* false_successor = instr->false_successor();
  impl().EnsureNativeBB(true_successor);
  impl().EnsureNativeBB(false_successor);
  ComparisonResolver resolver(impl());
  instr->comparison()->Accept(&resolver);
  LValue cmp_val = resolver.result();
  cmp_val = impl().EnsureBoolean(cmp_val);
  output().buildCondBr(cmp_val, impl().GetNativeBB(true_successor),
                       impl().GetNativeBB(false_successor));
  impl().EndCurrentBlock();
}

// FIXME: implement assertions.
void IRTranslator::VisitAssertAssignable(AssertAssignableInstr* instr) {}

void IRTranslator::VisitAssertSubtype(AssertSubtypeInstr* instr) {}

void IRTranslator::VisitAssertBoolean(AssertBooleanInstr* instr) {}

void IRTranslator::VisitSpecialParameter(SpecialParameterInstr* instr) {
  LValue val = nullptr;
  auto kind = instr->kind();
  if (kind == SpecialParameterInstr::kArgDescriptor) {
    val = output().args_desc();
  } else {
    EMASSERT(impl().current_bb_impl()->exception_block);
    switch (kind) {
      case SpecialParameterInstr::kException:
        val = impl().current_bb_impl()->exception_val;
        break;
      case SpecialParameterInstr::kStackTrace:
        val = impl().current_bb_impl()->stacktrace_val;
        break;
      default:
        UNREACHABLE();
    }
  }
  impl().SetLLVMValue(instr, val);
}

void IRTranslator::VisitClosureCall(ClosureCallInstr* instr) {
  impl().SetCurrentInstr(instr);
  const intptr_t argument_count =
      instr->ArgumentCount();  // Includes type args.
  LValue function =
      impl().GetLLVMValue(instr->InputAt(instr->InputCount() - 1));
  LValue entry = impl().LoadFieldFromOffset(
      function,
      compiler::target::Function::entry_point_offset(instr->entry_kind()),
      pointerType(output().repo().ref8));

  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
  callsite_info->set_token_pos(instr->token_pos());
  callsite_info->set_deopt_id(instr->deopt_id());
  callsite_info->set_stack_parameter_count(argument_count);
  callsite_info->set_instr_size(kCallInstrSize);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(impl(), instr->ssa_temp_index(), param);
  const Array& arguments_descriptor =
      Array::ZoneHandle(impl().zone(), instr->GetArgumentsDescriptor());
  LValue argument_descriptor_obj = impl().LoadObject(arguments_descriptor);
  resolver.SetGParameter(Output::RegisterToParameterLoc(ARGS_DESC_REG),
                         argument_descriptor_obj);
  resolver.SetCallTarget(entry);
  for (intptr_t i = argument_count - 1; i >= 0; --i) {
    LValue param = impl().GetLLVMValue(instr->ArgumentValueAt(i));
    resolver.AddStackParameter(param);
  }
  LValue result = resolver.BuildCall();
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitFfiCall(FfiCallInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitInstanceCallBase(InstanceCallBaseInstr* instr) {
  impl().SetCurrentInstr(instr);
  EMASSERT(instr->ic_data() != NULL);
  EMASSERT((FLAG_precompiled_mode && FLAG_use_bare_instructions));
  Zone* zone = impl().zone();
  ICData& ic_data = ICData::ZoneHandle(zone, instr->ic_data()->raw());
  ic_data = ic_data.AsUnaryClassChecks();
  if (instr->receiver_is_not_smi()) ic_data.set_receiver_cannot_be_smi(true);

  const intptr_t argument_count =
      instr->ArgumentCount();  // Includes type args.

  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  // use kReg type.
  callsite_info->set_type(CallSiteInfo::CallTargetType::kPatchable);
  callsite_info->set_token_pos(instr->token_pos());
  callsite_info->set_deopt_id(instr->deopt_id());
  callsite_info->set_stack_parameter_count(argument_count);
  callsite_info->set_kind(RawPcDescriptors::kIcCall);

  const Code& initial_stub = StubCode::UnlinkedCall();
  const UnlinkedCall& data =
      UnlinkedCall::ZoneHandle(zone, ic_data.AsUnlinkedCall());
  intptr_t ic_pool_offset, target_stub_pool_offset;
  ic_pool_offset = compiler::target::ObjectPool::element_offset(
      impl().object_pool_builder().AddObject(data));
  target_stub_pool_offset = compiler::target::ObjectPool::element_offset(
      impl().object_pool_builder().AddObject(initial_stub));
  callsite_info->set_ic_pool_offset(ic_pool_offset);
  callsite_info->set_target_stub_pool_offset(target_stub_pool_offset);
  size_t instr_size = kInstanceCallInstrSize;
  impl().AdjustInstrSizeForLoadFromConsecutiveAddress(instr_size,
                                                      ic_pool_offset);
  callsite_info->set_instr_size(instr_size);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(impl(), instr->ssa_temp_index(), param);
  for (intptr_t i = argument_count - 1; i >= 0; --i) {
    LValue param = impl().GetLLVMValue(instr->ArgumentValueAt(i));
    resolver.AddStackParameter(param);
  }
  LValue receiver =
      resolver.GetStackParameter((ic_data.CountWithoutTypeArgs() - 1));
  resolver.SetGParameter(Output::RegisterToParameterLoc(kReceiverReg),
                         receiver);

  LValue result = resolver.BuildCall();
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitInstanceCall(InstanceCallInstr* instr) {
  VisitInstanceCallBase(instr);
}

void IRTranslator::VisitPolymorphicInstanceCall(
    PolymorphicInstanceCallInstr* instr) {
  impl().SetCurrentInstr(instr);
  EMASSERT(!FLAG_polymorphic_with_deopt);
  EMASSERT(!instr->complete());
  VisitInstanceCallBase(instr);
}

void IRTranslator::VisitStaticCall(StaticCallInstr* instr) {
  impl().SetCurrentInstr(instr);
  Zone* zone = impl().zone();
  const ICData* call_ic_data = nullptr;
  if (instr->ic_data() == nullptr) {
    const Array& arguments_descriptor =
        Array::Handle(zone, instr->GetArgumentsDescriptor());
    const int num_args_checked =
        MethodRecognizer::NumArgsCheckedForStaticCall(instr->function());
    // FIXME: Check reusable
    // call_ic_data = compiler->GetOrAddStaticCallICData(
    //     instr->deopt_id(), instr->function(), arguments_descriptor, num_args_checked,
    //     instr->rebind_rule_);
    call_ic_data = &ICData::ZoneHandle(
        impl().zone(),
        ICData::New(impl().flow_graph()->parsed_function().function(),
                    String::Handle(impl().zone(), instr->function().name()),
                    arguments_descriptor, instr->deopt_id(), num_args_checked,
                    instr->rebind_rule_));
  } else {
    call_ic_data = &ICData::ZoneHandle(instr->ic_data()->raw());
  }
  ArgumentsInfo args_info(instr->type_args_len(), instr->ArgumentCount(),
                          instr->ArgumentsSize(), instr->argument_names());
  std::vector<LValue> arguments;
  for (intptr_t i = 0; i < args_info.count_with_type_args; ++i) {
    arguments.emplace_back(impl().GetLLVMValue(instr->ArgumentValueAt(i)));
  }
  LValue result = impl().GenerateStaticCall(
      instr, instr->ssa_temp_index(), arguments, instr->deopt_id(),
      instr->token_pos(), instr->function(), args_info, *call_ic_data,
      instr->entry_kind());
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitLoadLocal(LoadLocalInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitDropTemps(DropTempsInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitMakeTemp(MakeTempInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitStoreLocal(StoreLocalInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitStrictCompare(StrictCompareInstr* instr) {
  impl().SetCurrentInstr(instr);
  ComparisonResolver resolver(impl());
  instr->Accept(&resolver);
  LValue result = resolver.result();
  impl().SetLLVMValue(instr, impl().BooleanToObject(result));
}

void IRTranslator::VisitEqualityCompare(EqualityCompareInstr* instr) {
  impl().SetCurrentInstr(instr);
  ComparisonResolver resolver(impl());
  instr->Accept(&resolver);
  impl().SetLLVMValue(instr, impl().BooleanToObject(resolver.result()));
}

void IRTranslator::VisitRelationalOp(RelationalOpInstr* instr) {
  impl().SetCurrentInstr(instr);
  ComparisonResolver resolver(impl());
  instr->Accept(&resolver);
  impl().SetLLVMValue(instr, impl().BooleanToObject(resolver.result()));
}

void IRTranslator::VisitNativeCall(NativeCallInstr* instr) {
  impl().SetCurrentInstr(instr);
  EMASSERT(instr->link_lazily());
  const intptr_t argument_count =
      instr->ArgumentCount();  // Includes type args.

  const intptr_t argc_tag = NativeArguments::ComputeArgcTag(instr->function());

  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kNative);
  callsite_info->set_token_pos(instr->token_pos());
  callsite_info->set_deopt_id(instr->deopt_id());
  callsite_info->set_stack_parameter_count(argument_count);
  size_t instr_size = kNativeCallInstrSize;
  int32_t native_entry_pool_offset, stub_pool_offset;
  uword entry;
  const Code* stub;
  stub = &StubCode::CallBootstrapNative();
  entry = NativeEntry::LinkNativeCallEntry();
  compiler::ExternalLabel label(entry);
  native_entry_pool_offset = compiler::target::ObjectPool::element_offset(
      impl().object_pool_builder().FindNativeFunction(
          &label, compiler::ObjectPoolBuilderEntry::kPatchable));
  stub_pool_offset = compiler::target::ObjectPool::element_offset(
      impl().object_pool_builder().FindObject(
          *stub, compiler::ObjectPoolBuilderEntry::kPatchable));

  impl().AdjustInstrSizeForLoad(instr_size, native_entry_pool_offset);
  impl().AdjustInstrSizeForLoad(instr_size, stub_pool_offset);
  callsite_info->set_native_entry_pool_offset(native_entry_pool_offset);
  callsite_info->set_stub_pool_offset(stub_pool_offset);
  callsite_info->set_instr_size(instr_size);
  callsite_info->set_return_on_stack_pos(0);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(impl(), instr->ssa_temp_index(), param);
  resolver.SetGParameter(Output::RegisterToParameterLoc(kNativeArgcReg),
                         output().constIntPtr(argc_tag));
  resolver.AddStackParameter(impl().GetNull());
  for (intptr_t i = argument_count - 1; i >= 0; --i) {
    LValue argument = impl().GetLLVMValue(instr->ArgumentValueAt(i));
    resolver.AddStackParameter(argument);
  }
  impl().SetLLVMValue(instr, resolver.BuildCall());
}

void IRTranslator::VisitDebugStepCheck(DebugStepCheckInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitLoadIndexed(LoadIndexedInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue array = impl().GetLLVMValue(instr->array());
  LValue index = impl().GetLLVMValue(instr->index());
  EMASSERT(typeOf(index) == output().tagged_type());
  index = impl().TaggedToWord(index);
  // Found out the input representation for index.
  Representation index_rep = instr->RequiredInputRepresentation(1);

  const intptr_t boxing_shift = index_rep != kTagged ? 0 : -kSmiTagShift;
  const intptr_t shift =
      Utils::ShiftForPowerOfTwo(instr->index_scale()) + boxing_shift;
  int32_t offset =
      instr->IsExternal()
          ? 0
          : (compiler::target::Instance::DataOffsetFor(instr->class_id()) -
             kHeapObjectTag);
  if (shift < 0) {
    EMASSERT(shift == -1);
    index = output().buildSar(index, output().constIntPtr(1));
  } else {
    index = output().buildShl(index, output().constIntPtr(shift));
  }
  LValue offset_value = output().buildAdd(output().constIntPtr(offset), index);
  LValue val;
  if ((instr->representation() == kUnboxedDouble) ||
      (instr->representation() == kUnboxedFloat32x4) ||
      (instr->representation() == kUnboxedInt32x4) ||
      (instr->representation() == kUnboxedFloat64x2)) {
    switch (instr->class_id()) {
      case kTypedDataFloat32ArrayCid:
        // Load single precision float.
        // vldrs does not support indexed addressing.
        if (instr->aligned()) {
          LValue gep =
              impl().BuildAccessPointer(array, offset_value, kUnboxedFloat);
          val = output().buildLoad(gep);
        } else {
          LValue gep =
              impl().BuildAccessPointer(array, offset_value, kUnboxedFloat);
          val = output().buildLoadUnaligned(gep);
        }
        break;
      case kTypedDataFloat64ArrayCid:
        // vldrd does not support indexed addressing.
        if (instr->aligned()) {
          LValue gep =
              impl().BuildAccessPointer(array, offset_value, kUnboxedDouble);
          val = output().buildLoad(gep);
        } else {
          LValue gep =
              impl().BuildAccessPointer(array, offset_value, kUnboxedDouble);
          val = output().buildLoadUnaligned(gep);
        }
        break;
      case kTypedDataFloat64x2ArrayCid: {
        EMASSERT(instr->aligned());
        LValue gep =
            impl().BuildAccessPointer(array, offset_value, kUnboxedFloat64x2);
        val = output().buildLoad(gep);
        break;
      }
      case kTypedDataInt32x4ArrayCid: {
        EMASSERT(instr->aligned());
        LValue gep =
            impl().BuildAccessPointer(array, offset_value, kUnboxedInt32x4);
        val = output().buildLoad(gep);
        break;
      }
      case kTypedDataFloat32x4ArrayCid: {
        EMASSERT(instr->aligned());
        LValue gep =
            impl().BuildAccessPointer(array, offset_value, kUnboxedFloat32x4);
        val = output().buildLoad(gep);
        break;
      }

      default:
        UNREACHABLE();
    }
    impl().SetLLVMValue(instr, val);
    return;
  }
  switch (instr->class_id()) {
    case kTypedDataUint32ArrayCid:
    case kTypedDataInt32ArrayCid: {
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, kUnboxedInt32);
      val = output().buildLoad(gep);
      break;
    }
    case kTypedDataInt64ArrayCid:
    case kTypedDataUint64ArrayCid: {
      EMASSERT(instr->representation() == kUnboxedInt64);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, kUnboxedInt64);
      val = output().buildLoad(gep);
      break;
    }
    case kTypedDataInt8ArrayCid: {
      EMASSERT(instr->representation() == kUnboxedIntPtr);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref8);
      val = output().buildLoad(gep);
      val = output().buildCast(LLVMSExt, val, output().repo().intPtr);
      break;
    }
    case kTypedDataUint8ArrayCid:
    case kTypedDataUint8ClampedArrayCid:
    case kExternalTypedDataUint8ArrayCid:
    case kExternalTypedDataUint8ClampedArrayCid:
    case kOneByteStringCid:
    case kExternalOneByteStringCid: {
      EMASSERT(instr->representation() == kUnboxedIntPtr);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref8);
      val = output().buildLoad(gep);
      val = output().buildCast(LLVMZExt, val, output().repo().intPtr);
      break;
    }
    case kTypedDataInt16ArrayCid: {
      EMASSERT(instr->representation() == kUnboxedIntPtr);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref16);
      val = output().buildLoad(gep);
      val = output().buildCast(LLVMSExt, val, output().repo().intPtr);
      break;
    }
    case kTypedDataUint16ArrayCid:
    case kTwoByteStringCid:
    case kExternalTwoByteStringCid: {
      EMASSERT(instr->representation() == kUnboxedIntPtr);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref16);
      val = output().buildLoad(gep);
      val = output().buildCast(LLVMZExt, val, output().repo().intPtr);
      break;
    }
    default: {
      EMASSERT(instr->representation() == kTagged);
      EMASSERT((instr->class_id() == kArrayCid) ||
               (instr->class_id() == kImmutableArrayCid));
      LValue gep = impl().BuildAccessPointer(array, offset_value, kTagged);
      val = output().buildLoad(gep);
      break;
    }
  }
  impl().SetLLVMValue(instr, val);
}

void IRTranslator::VisitLoadCodeUnits(LoadCodeUnitsInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue array = impl().GetLLVMValue(instr->array());
  LValue index = impl().GetLLVMValue(instr->index());
  Representation index_rep = instr->RequiredInputRepresentation(1);
  const intptr_t boxing_shift = index_rep != kTagged ? 0 : -kSmiTagShift;
  const intptr_t shift =
      Utils::ShiftForPowerOfTwo(instr->index_scale()) + boxing_shift;
  int32_t offset =
      instr->IsExternal()
          ? 0
          : (compiler::target::Instance::DataOffsetFor(instr->class_id()) -
             kHeapObjectTag);
  if (shift < 0) {
    EMASSERT(shift == -1);
    index = output().buildSar(index, output().constIntPtr(1));
  } else {
    index = output().buildShl(index, output().constIntPtr(shift));
  }
  LValue offset_value = output().buildAdd(output().constIntPtr(offset), index);
  LValue value;
#if !defined(TARGET_ARCH_IS_64_BIT)
  if (instr->representation() == kUnboxedInt64) {
    LValue gep = impl().BuildAccessPointer(array, offset_value, kUnboxedUint32);
    LValue value_32 = output().buildLoad(gep);
    value = output().buildCast(LLVMZExt, value_32, output().repo().int64);
  }
#endif
  EMASSERT(instr->representation() == kTagged);
  LType type;
  switch (instr->class_id()) {
    case kOneByteStringCid:
    case kExternalOneByteStringCid:
      switch (instr->element_count()) {
        case 1:
          type = output().repo().int8;
          break;
        case 2:
          type = output().repo().int16;
          break;
        case 4:
          type = output().repo().int32;
          break;
        default:
          UNREACHABLE();
      }
      break;
    case kTwoByteStringCid:
    case kExternalTwoByteStringCid:
      switch (instr->element_count()) {
        case 1:
          type = output().repo().int16;
          break;
        case 2:
          type = output().repo().int32;
          break;
        default:
          UNREACHABLE();
      }
      break;
    default:
      UNREACHABLE();
      break;
  }
  LValue gep =
      impl().BuildAccessPointer(array, offset_value, pointerType(type));
  LValue int_value = impl().EnsureIntPtr(output().buildLoad(gep));

  if (instr->can_pack_into_smi()) {
    value = impl().SmiTag(int_value);
  } else {
#if defined(TARGET_ARCH_IS_32_BIT)
    DiamondContinuationResolver diamond(impl(), instr->ssa_temp_index());
    diamond.LeftHint();
    diamond.BuildCmp([&]() {
      LValue and_value =
          output().buildAnd(int_value, output().constIntPtr(0xC0000000));
      LValue cmp =
          output().buildICmp(LLVMIntEQ, and_value, output().constIntPtr(0));
      return cmp;
    });
    diamond.BuildLeft([&]() { return impl().SmiTag(int_value); });
    diamond.BuildRight([&]() {
      BoxAllocationSlowPath allocation(instr, impl().mint_class(), impl());
      return allocation.Allocate();
    });
    value = diamond.End();
#else
    UNREACHABLE();
#endif
  }
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitStoreIndexed(StoreIndexedInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue array = impl().GetLLVMValue(instr->array());
  LValue index = impl().TaggedToWord(impl().GetLLVMValue(instr->index()));
  LValue value = impl().GetLLVMValue(instr->value());
  Representation index_rep = instr->RequiredInputRepresentation(1);
  const intptr_t boxing_shift = index_rep != kTagged ? 0 : -kSmiTagShift;
  const intptr_t shift =
      Utils::ShiftForPowerOfTwo(instr->index_scale()) + boxing_shift;
  int32_t offset =
      instr->IsExternal()
          ? 0
          : (compiler::target::Instance::DataOffsetFor(instr->class_id()) -
             kHeapObjectTag);
  if (shift < 0) {
    EMASSERT(shift == -1);
    index = output().buildSar(index, output().constIntPtr(1));
  } else {
    index = output().buildShl(index, output().constIntPtr(shift));
  }
  LValue offset_value = output().buildAdd(output().constIntPtr(offset), index);
  switch (instr->class_id()) {
    case kArrayCid:
      if (instr->ShouldEmitStoreBarrier()) {
        impl().StoreIntoArray(instr, array, offset_value, value,
                              instr->CanValueBeSmi());
      } else {
        LValue gep = impl().BuildAccessPointer(
            array, offset_value, pointerType(output().tagged_type()));
        output().buildStore(value, gep);
      }
      break;
    case kTypedDataInt8ArrayCid:
    case kTypedDataUint8ArrayCid:
    case kExternalTypedDataUint8ArrayCid:
    case kOneByteStringCid: {
      EMASSERT(instr->RequiredInputRepresentation(2) == kUnboxedIntPtr);
      LValue value_to_store =
          output().buildCast(LLVMTrunc, value, output().repo().int8);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref8);
      output().buildStore(value_to_store, gep);
      break;
    }
    case kTypedDataUint8ClampedArrayCid:
    case kExternalTypedDataUint8ClampedArrayCid: {
      EMASSERT(instr->RequiredInputRepresentation(2) == kUnboxedIntPtr);
      value = impl().EnsureIntPtr(value);
      LValue constant = output().constIntPtr(0xFF);
      LValue cmp_1 = output().buildICmp(LLVMIntSLE, value, constant);
      LValue cmp_2 = output().buildICmp(LLVMIntULE, value, constant);
      LValue value_select =
          output().buildSelect(cmp_1, output().constIntPtr(0), constant);
      value = output().buildSelect(cmp_2, value, value_select);
      LValue value_to_store =
          output().buildCast(LLVMTrunc, value, output().repo().int8);
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref8);
      output().buildStore(value_to_store, gep);
      break;
    }
    case kTypedDataInt16ArrayCid:
    case kTypedDataUint16ArrayCid: {
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref16);
      if (typeOf(value) != output().repo().int16)
        value = output().buildCast(LLVMTrunc, value, output().repo().int16);
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataInt32ArrayCid:
    case kTypedDataUint32ArrayCid: {
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref32);
      if (typeOf(value) != output().repo().int32)
        value = output().buildCast(LLVMTrunc, value, output().repo().int32);
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataInt64ArrayCid:
    case kTypedDataUint64ArrayCid: {
      LValue gep =
          impl().BuildAccessPointer(array, offset_value, output().repo().ref64);
      EMASSERT(typeOf(value) == output().repo().int64)
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataFloat32ArrayCid: {
      LValue gep = impl().BuildAccessPointer(array, offset_value,
                                             output().repo().refFloat);
      EMASSERT(typeOf(value) == output().repo().floatType)
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataFloat64ArrayCid: {
      LValue gep = impl().BuildAccessPointer(array, offset_value,
                                             output().repo().refDouble);
      EMASSERT(typeOf(value) == output().repo().doubleType);
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataFloat64x2ArrayCid: {
      LValue gep = impl().BuildAccessPointer(
          array, offset_value, pointerType(output().repo().float64x2));
      EMASSERT(typeOf(value) == output().repo().float64x2);
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataInt32x4ArrayCid: {
      LValue gep = impl().BuildAccessPointer(
          array, offset_value, pointerType(output().repo().int32x4));
      EMASSERT(typeOf(value) == output().repo().int32x4);
      output().buildStore(value, gep);
      break;
    }
    case kTypedDataFloat32x4ArrayCid: {
      LValue gep = impl().BuildAccessPointer(
          array, offset_value, pointerType(output().repo().float32x4));
      EMASSERT(typeOf(value) == output().repo().float32x4);
      output().buildStore(value, gep);
      break;
    }

    default:
      UNREACHABLE();
  }
}

void IRTranslator::VisitStoreInstanceField(StoreInstanceFieldInstr* instr) {
  impl().SetCurrentInstr(instr);
  const intptr_t offset_in_bytes = instr->OffsetInBytes();

  LValue val = impl().GetLLVMValue(instr->value());
  LValue instance = impl().GetLLVMValue(instr->instance());

  if (instr->IsUnboxedStore()) {
    impl().StoreToOffset(instance, offset_in_bytes - kHeapObjectTag, val);
    if (instr->slot().field().is_non_nullable_integer()) {
      EMASSERT(typeOf(val) == output().repo().int64);
      return;
    }

    const intptr_t cid = instr->slot().field().UnboxedFieldCid();
    switch (cid) {
      case kDoubleCid:
        EMASSERT(typeOf(val) == output().repo().doubleType);
        return;
      case kFloat32x4Cid:
        LLVMLOGE("StoreInstanceFieldInstr: Unsupport kFloat32x4Cid\n");
        UNREACHABLE();
      case kFloat64x2Cid:
        LLVMLOGE("StoreInstanceFieldInstr: Unsupport kFloat64x2Cid\n");
        UNREACHABLE();
      default:
        UNREACHABLE();
    }
  }

  if (instr->IsPotentialUnboxedStore()) {
    LLVMLOGE("Unsupported IsPotentialUnboxedStore\n");
    UNREACHABLE();
  }
  if (instr->ShouldEmitStoreBarrier()) {
    impl().StoreIntoObject(
        instr, instance, output().constIntPtr(offset_in_bytes - kHeapObjectTag),
        val, instr->CanValueBeSmi());
  } else {
    impl().StoreToOffset(instance, offset_in_bytes - kHeapObjectTag, val);
  }
}

void IRTranslator::VisitInitInstanceField(InitInstanceFieldInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue instance = impl().GetLLVMValue(instr->InputAt(0));
  LValue field_value =
      impl().LoadFieldFromOffset(instance, instr->field().TargetOffset());
  LValue sentinel = impl().LoadObject(Object::sentinel());
  AssemblerResolver resolver(impl());
  LValue cmp = output().buildICmp(LLVMIntNE, field_value, sentinel);
  resolver.GotoMergeIf(impl().ExpectTrue(cmp));

  LValue field_original =
      impl().LoadObject(Field::ZoneHandle(instr->field().Original()));
  impl().PushArgument(instance);
  impl().PushArgument(field_original);
  impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                             kInitInstanceFieldRuntimeEntry, 2);
  resolver.GotoMerge();
  resolver.End();
}

void IRTranslator::VisitInitStaticField(InitStaticFieldInstr* instr) {
  impl().SetCurrentInstr(instr);
  const intptr_t field_table_offset =
      compiler::target::Thread::field_table_values_offset();
  const intptr_t field_offset =
      compiler::target::FieldTable::OffsetOf(instr->field());
  Label call_runtime("call_runtime");
  LValue field_table =
      impl().LoadFromOffset(output().thread(), field_table_offset,
                            pointerType(output().repo().ref8), true);
  LValue field = impl().LoadFromOffset(field_table, field_offset,
                                       pointerType(output().tagged_type()));
  LValue sentinel = impl().LoadObject(Object::sentinel());
  AssemblerResolver resolver(impl());
  LValue cmp = output().buildICmp(LLVMIntEQ, field, sentinel);
  resolver.BranchIf(impl().ExpectFalse(cmp), call_runtime);

  LValue transition_sentinel = impl().LoadObject(Object::transition_sentinel());
  cmp = output().buildICmp(LLVMIntNE, field, transition_sentinel);
  resolver.GotoMergeIf(impl().ExpectTrue(cmp));
  resolver.Branch(call_runtime);

  resolver.Bind(call_runtime);
  LValue field_original =
      impl().LoadObject(Field::ZoneHandle(instr->field().Original()));
  constexpr const Register kFieldOrignalReg = InitStaticFieldABI::kFieldReg;
  impl().GenerateCall(instr, instr->token_pos(), instr->deopt_id(),
                      StubCode::InitStaticField(), RawPcDescriptors::kOther, 0,
                      {{kFieldOrignalReg, field_original}});
  resolver.GotoMerge();
  resolver.End();
}

void IRTranslator::VisitLoadStaticField(LoadStaticFieldInstr* instr) {
  impl().SetCurrentInstr(instr);

  LValue field_table_values = impl().LoadFromOffset(
      output().thread(), compiler::target::Thread::field_table_values_offset(),
      pointerType(output().repo().ref8), true);

  LValue v = impl().LoadFromOffset(
      field_table_values,
      compiler::target::FieldTable::OffsetOf(instr->StaticField()),
      pointerType(output().tagged_type()));
  impl().SetLLVMValue(instr, v);
}

void IRTranslator::VisitStoreStaticField(StoreStaticFieldInstr* instr) {
  impl().SetCurrentInstr(instr);
  const intptr_t field_table_offset =
      compiler::target::Thread::field_table_values_offset();
  LValue field_table =
      impl().LoadFromOffset(output().thread(), field_table_offset,
                            pointerType(output().repo().ref8), true);
  LValue value = impl().GetLLVMValue(instr->value());
  impl().StoreToOffset(field_table,
                       compiler::target::FieldTable::OffsetOf(instr->field()),
                       value);
}

void IRTranslator::VisitBooleanNegate(BooleanNegateInstr* instr) {
  impl().SetCurrentInstr(instr);

  LValue true_or_false = impl().GetLLVMValue(instr->value());
  LValue true_value = impl().LoadObject(Bool::True());
  LValue false_value = impl().LoadObject(Bool::False());
  LValue cmp_value = output().buildICmp(LLVMIntEQ, true_value, true_or_false);
  LValue value = output().buildSelect(cmp_value, false_value, true_value);
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitInstanceOf(InstanceOfInstr* instr) {
  impl().SetCurrentInstr(instr);
  // FIXME: implement inline instance of
  LValue instance = impl().GetLLVMValue(instr->value());
  LValue instantiator_type_arguments =
      impl().GetLLVMValue(instr->instantiator_type_arguments());
  LValue function_type_arguments =
      impl().GetLLVMValue(instr->function_type_arguments());
  LValue null_object = impl().GetNull();
  impl().PushArgument(instance);
  impl().PushArgument(impl().LoadObject(instr->type()));
  impl().PushArgument(instantiator_type_arguments);
  impl().PushArgument(function_type_arguments);
  impl().PushArgument(null_object);  // cache
  LValue result = impl().GenerateRuntimeCall(
      instr, instr->token_pos(), instr->deopt_id(), kInstanceofRuntimeEntry, 5);
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitCreateArray(CreateArrayInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue length = impl().GetLLVMValue(instr->num_elements());
  LValue elem_type = impl().GetLLVMValue(instr->element_type());
  LValue result = impl().GenerateCall(
      instr, instr->token_pos(), instr->deopt_id(), StubCode::AllocateArray(),
      RawPcDescriptors::kOther, 0,
      {{kCreateArrayLengthReg, length},
       {kCreateArrayElementTypeReg, elem_type}});
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitAllocateObject(AllocateObjectInstr* instr) {
  impl().SetCurrentInstr(instr);
  std::vector<std::pair<Register, LValue>> args;
  if (instr->type_arguments() != nullptr) {
    TypeUsageInfo* type_usage_info = impl().thread()->type_usage_info();
    if (type_usage_info != nullptr) {
      RegisterTypeArgumentsUse(
          impl().flow_graph()->parsed_function().function(), type_usage_info,
          instr->cls(), instr->type_arguments()->definition());
    }
    args.emplace_back(kAllocationStubTypeArgumentsReg,
                      impl().GetLLVMValue(instr->type_arguments()));
  }
  const Code& stub = Code::ZoneHandle(
      impl().zone(), StubCode::GetAllocationStubForClass(instr->cls()));

  LValue result = impl().GenerateCall(
      instr, instr->token_pos(), instr->deopt_id(), stub,
      RawPcDescriptors::kOther, instr->ArgumentCount(), args);
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitLoadField(LoadFieldInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue instance = impl().GetLLVMValue(instr->instance());
  intptr_t offset_in_bytes = instr->slot().offset_in_bytes();
  if (instr->IsUnboxedLoad()) {
    if (instr->slot().field().is_non_nullable_integer()) {
      LValue value = impl().LoadFieldFromOffset(instance, offset_in_bytes,
                                                output().repo().ref64);
      impl().SetLLVMValue(instr, value);
      return;
    }
    const intptr_t cid = instr->slot().field().UnboxedFieldCid();
    LValue value;
    EMASSERT(FLAG_precompiled_mode && "LLVM must in precompiled mode");
    switch (cid) {
      case kDoubleCid:
        value = impl().LoadFieldFromOffset(instance, offset_in_bytes,
                                           output().repo().refDouble);
        ;
        break;
      case kFloat32x4Cid:
        EMASSERT("kFloat32x4Cid not supported" && false);
        UNREACHABLE();
      case kFloat64x2Cid:
        EMASSERT("kFloat64x2Cid not supported" && false);
        UNREACHABLE();
      default:
        UNREACHABLE();
    }
    impl().SetLLVMValue(instr, value);
    return;
  }
  if (instr->IsPotentialUnboxedLoad()) {
    // FIXME: need for continuation resolver
    // support after call implementation.
    UNREACHABLE();
  }
  LValue value;
  if (instr->HasType() && instr->Type()->ToCid() == kSmiCid)
    value = impl().LoadSmiFieldFromOffset(instance, offset_in_bytes);
  else
    value = impl().LoadFieldFromOffset(instance, offset_in_bytes);
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitLoadUntagged(LoadUntaggedInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue object = impl().GetLLVMValue(instr->object());
  LValue value;
  if (instr->object()->definition()->representation() == kUntagged) {
    value =
        impl().LoadFromOffset(object, instr->offset(), output().repo().refPtr);
  } else {
    value = impl().LoadFieldFromOffset(object, instr->offset());
  }
  impl().SetLLVMValue(instr, impl().TaggedToWord(value));
}

void IRTranslator::VisitStoreUntagged(StoreUntaggedInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue object = impl().GetLLVMValue(instr->object());
  LValue value = impl().GetLLVMValue(instr->value());
  impl().StoreToOffset(object, instr->offset_from_tagged(), value);
}

void IRTranslator::VisitLoadClassId(LoadClassIdInstr* instr) {
  impl().SetCurrentInstr(instr);
  const AbstractType& value_type = *instr->object()->Type()->ToAbstractType();
  LValue object = impl().GetLLVMValue(instr->object());
  LValue value;
  if (instr->input_can_be_smi_ &&
      (CompileType::Smi().IsAssignableTo(value_type) ||
       value_type.IsTypeParameter())) {
    DiamondContinuationResolver diamond(impl(), instr->ssa_temp_index());
    diamond.BuildCmp([&]() { return impl().IsSmi(object); })
        .BuildLeft([&]() { return output().constInt16(kSmiCid); })
        .BuildRight([&]() { return impl().LoadClassId(object); });

    value = diamond.End();
  } else {
    value = impl().LoadClassId(object);
  }
  if (instr->representation() == kTagged)
    value = impl().SmiTag(
        output().buildCast(LLVMZExt, value, output().repo().intPtr));
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitInstantiateType(InstantiateTypeInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue instantiator_type_args =
      impl().GetLLVMValue(instr->instantiator_type_arguments());
  LValue function_type_args =
      impl().GetLLVMValue(instr->function_type_arguments());
  impl().PushArgument(impl().LoadObject(instr->type()));
  impl().PushArgument(instantiator_type_args);
  impl().PushArgument(function_type_args);
  LValue result =
      impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                                 kInstantiateTypeRuntimeEntry, 3);
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitInstantiateTypeArguments(
    InstantiateTypeArgumentsInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue instantiator_type_args =
      impl().GetLLVMValue(instr->instantiator_type_arguments());
  LValue function_type_args =
      impl().GetLLVMValue(instr->function_type_arguments());
  AssemblerResolver resolver(impl());
  Label type_arguments_instantiated("type_arguments_instantiated");
  // If both the instantiator and function type arguments are null and if the
  // type argument vector instantiated from null becomes a vector of dynamic,
  // then use null as the type arguments.
  const intptr_t len = instr->type_arguments().Length();
  if (instr->type_arguments().IsRawWhenInstantiatedFromRaw(len)) {
    LValue null_object = impl().GetNull();
    LValue cmp_1 =
        output().buildICmp(LLVMIntEQ, instantiator_type_args, null_object);
    LValue cmp_2 =
        output().buildICmp(LLVMIntEQ, function_type_args, null_object);
    LValue and_val = output().buildAnd(cmp_1, cmp_2);
    resolver.GotoMergeWithValueIf(and_val, null_object);
  }
  constexpr Register kInstantiatorTypeArgumentsReg =
      InstantiationABI::kInstantiatorTypeArgumentsReg;
  constexpr Register kFunctionTypeArgumentsReg =
      InstantiationABI::kFunctionTypeArgumentsReg;
  constexpr Register kUninstantiatedTypeArgumentsReg =
      InstantiationABI::kUninstantiatedTypeArgumentsReg;
  LValue result = impl().GenerateCall(
      instr, instr->token_pos(), instr->deopt_id(), instr->GetStub(),
      RawPcDescriptors::kOther, 0,
      {{kInstantiatorTypeArgumentsReg, instantiator_type_args},
       {kFunctionTypeArgumentsReg, function_type_args},
       {kUninstantiatedTypeArgumentsReg,
        impl().LoadObject(instr->type_arguments())}});

  resolver.GotoMergeWithValue(result);
  result = resolver.End();
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitAllocateContext(AllocateContextInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue result = impl().GenerateCall(
      instr, instr->token_pos(), instr->deopt_id(), StubCode::AllocateContext(),
      RawPcDescriptors::kOther, 0,
      {{kAllocateContextNumOfContextVarsReg,
        output().constIntPtr(instr->num_context_variables())}});
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitAllocateUninitializedContext(
    AllocateUninitializedContextInstr* instr) {
  intptr_t instance_size =
      Context::InstanceSize(instr->num_context_variables());
  AssemblerResolver resolver(impl());
  Label slow_path("AllocateContextSlowPath:%d", instr->ssa_temp_index());
  LValue instance =
      impl().TryAllocate(resolver, slow_path, kContextCid, instance_size);
  if (instance) {
    impl().StoreToOffset(
        instance,
        compiler::target::Context::num_variables_offset() - kHeapObjectTag,
        output().constIntPtr(instr->num_context_variables()));
    resolver.GotoMergeWithValue(instance);
  }
  resolver.Bind(slow_path);
  resolver.GotoMergeWithValue(impl().GenerateCall(
      instr, instr->token_pos(), instr->deopt_id(), StubCode::AllocateContext(),
      RawPcDescriptors::kOther, 0,
      {{kAllocateContextNumOfContextVarsReg,
        output().constIntPtr(instr->num_context_variables())}}));
  LValue result = resolver.End();
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitCloneContext(CloneContextInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue context_value = impl().GetLLVMValue(instr->context_value());
  impl().PushArgument(context_value);
  LValue result =
      impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                                 kCloneContextRuntimeEntry, 1);
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitBinarySmiOp(BinarySmiOpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value;
  if (instr->right()->definition()->IsConstant()) {
    ConstantInstr* constant = instr->right()->definition()->AsConstant();
    const intptr_t imm = compiler::target::ToRawSmi(constant->value());
    LValue left = impl().TaggedToWord(impl().GetLLVMValue(instr->left()));
    switch (instr->op_kind()) {
      case Token::kADD: {
        value = output().buildAdd(left, output().constIntPtr(imm));
        value = impl().WordToTagged(value);
        break;
      }
      case Token::kSUB: {
        value = output().buildSub(left, output().constIntPtr(imm));
        value = impl().WordToTagged(value);
        break;
      }
      case Token::kMUL: {
        // Keep left value tagged and untag right value.
        const intptr_t actual_value =
            compiler::target::SmiValue(constant->value());
        value = output().buildMul(left, output().constIntPtr(actual_value));
        value = impl().WordToTagged(value);
        break;
      }
      case Token::kTRUNCDIV: {
        const intptr_t cvalue = compiler::target::SmiValue(constant->value());
        EMASSERT(cvalue != kIntptrMin);
        EMASSERT(Utils::IsPowerOfTwo(Utils::Abs(cvalue)));
        const intptr_t shift_count =
            Utils::ShiftForPowerOfTwo(Utils::Abs(cvalue)) + kSmiTagSize;
        LValue left = impl().TaggedToWord(impl().GetLLVMValue(instr->left()));
        LValue left_asr = output().buildSar(left, output().constIntPtr(31));
        LValue temp = output().buildAdd(
            left, output().buildShr(left_asr,
                                    output().constIntPtr(32 - shift_count)));
        value = output().buildSar(temp, output().constIntPtr(shift_count));
        if (cvalue < 0)
          value = output().buildSub(output().constIntPtr(0), value);
        value = impl().SmiTag(value);
      } break;
      case Token::kBIT_AND: {
        value = output().buildAnd(left, output().constIntPtr(imm));
        value = impl().WordToTagged(value);
        break;
      }
      case Token::kBIT_OR: {
        value = output().buildOr(left, output().constIntPtr(imm));
        value = impl().WordToTagged(value);
        break;
      }
      case Token::kBIT_XOR: {
        value = output().buildXor(left, output().constIntPtr(imm));
        value = impl().WordToTagged(value);
        break;
      }
      case Token::kSHR: {
        // sarl operation masks the count to 5 bits.
        const intptr_t kCountLimit = 0x1F;
        intptr_t actual_value = compiler::target::SmiValue(constant->value());
        value = output().buildSar(
            left, output().constIntPtr(
                      Utils::Minimum(actual_value + kSmiTagSize, kCountLimit)));
        value = impl().SmiTag(value);
        break;
      }
      case Token::kSHL: {
        const intptr_t actual_value =
            compiler::target::SmiValue(constant->value());
        value = output().buildShl(left, output().constIntPtr(actual_value));
        value = impl().WordToTagged(value);
        break;
      }

      default:
        UNREACHABLE();
        break;
    }
  } else {
    LValue left = impl().SmiUntag(impl().GetLLVMValue(instr->left()));
    LValue right = impl().SmiUntag(impl().GetLLVMValue(instr->right()));
    output().assignEvenNumberAttr(left);
    output().assignEvenNumberAttr(right);
    switch (instr->op_kind()) {
      case Token::kSHL:
        value = output().buildShl(left, right);
        break;
      case Token::kADD:
        value = output().buildAdd(left, right);
        break;
      case Token::kSUB:
        value = output().buildSub(left, right);
        break;
      case Token::kMUL:
        value = output().buildMul(left, right);
        break;
      case Token::kTRUNCDIV: {
        bool support_int_div = true;
        support_int_div = impl().support_integer_div();
        if (support_int_div) {
          value = output().buildSDiv(left, right);
        } else {
          LValue left_double =
              output().buildCast(LLVMSIToFP, left, output().repo().doubleType);
          LValue right_double =
              output().buildCast(LLVMSIToFP, right, output().repo().doubleType);
          LValue value_double = output().buildFDiv(left_double, right_double);
          value = output().buildCast(LLVMFPToSI, value_double,
                                     output().repo().intPtr);
        }
      } break;
      case Token::kBIT_AND:
        value = output().buildAnd(left, right);
        break;
      case Token::kBIT_OR:
        value = output().buildOr(left, right);
        break;
      case Token::kBIT_XOR:
        value = output().buildXor(left, right);
        break;
      case Token::kSHR:
        value = output().buildShr(left, right);
        break;
      default:
        UNREACHABLE();
        break;
    }
    value = impl().SmiTag(value);
  }
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitCheckedSmiComparison(CheckedSmiComparisonInstr* instr) {
  impl().SetCurrentInstr(instr);
  ComparisonResolver resolver(impl());
  instr->Accept(&resolver);
  impl().SetLLVMValue(instr, impl().BooleanToObject(resolver.result()));
}

void IRTranslator::VisitCheckedSmiOp(CheckedSmiOpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue left = impl().GetLLVMValue(instr->InputAt(0));
  LValue right = impl().GetLLVMValue(instr->InputAt(1));

  intptr_t left_cid = instr->left()->Type()->ToCid();
  intptr_t right_cid = instr->right()->Type()->ToCid();
  DiamondContinuationResolver diamond(impl(), instr->ssa_temp_index());
  diamond.LeftHint();
  diamond.BuildCmp([&] {
    LValue is_smi;
    if (instr->left()->definition() == instr->right()->definition()) {
      is_smi = impl().IsSmi(left);
    } else if (left_cid == kSmiCid) {
      is_smi = impl().IsSmi(right);
    } else if (right_cid == kSmiCid) {
      is_smi = impl().IsSmi(left);
    } else {
      LValue left_address = impl().TaggedToWord(left);
      LValue right_address = impl().TaggedToWord(right);
      LValue orr_address = output().buildOr(left_address, right_address);
      LValue and_value =
          output().buildAnd(orr_address, output().constIntPtr(kSmiTagMask));
      is_smi = output().buildICmp(LLVMIntNE, and_value,
                                  output().constIntPtr(kSmiTagMask));
    }
    return is_smi;
  });
  diamond.BuildLeft([&] {
    LValue result;
    switch (instr->op_kind()) {
      case Token::kADD: {
#if defined(TARGET_ARCH_IS_32_BIT)
        LValue add_result = output().buildCall(
            output().repo().addWithOverflow32Intrinsic(),
            impl().TaggedToWord(left), impl().TaggedToWord(right));
#else
        LValue add_result = output().buildCall(
            output().repo().addWithOverflow64Intrinsic(),
            impl().TaggedToWord(left), impl().TaggedToWord(right));
#endif
        diamond.BranchToOtherLeafOnCondition(
            output().buildExtractValue(add_result, 1));
        result = impl().WordToTagged(output().buildExtractValue(add_result, 0));
      } break;
      case Token::kSUB: {
#if defined(TARGET_ARCH_IS_32_BIT)
        LValue sub_result = output().buildCall(
            output().repo().subWithOverflow32Intrinsic(),
            impl().TaggedToWord(left), impl().TaggedToWord(right));
#else
        LValue sub_result = output().buildCall(
            output().repo().subWithOverflow64Intrinsic(),
            impl().TaggedToWord(left), impl().TaggedToWord(right));
#endif
        diamond.BranchToOtherLeafOnCondition(
            output().buildExtractValue(sub_result, 1));
        result = impl().WordToTagged(output().buildExtractValue(sub_result, 0));
      } break;
      case Token::kMUL: {
#if defined(TARGET_ARCH_IS_32_BIT)
        LValue mul_result = output().buildCall(
            output().repo().mulWithOverflow32Intrinsic(), impl().SmiUntag(left),
            impl().TaggedToWord(right));
#else
        LValue mul_result = output().buildCall(
            output().repo().mulWithOverflow64Intrinsic(), impl().SmiUntag(left),
            impl().TaggedToWord(right));
#endif
        diamond.BranchToOtherLeafOnCondition(
            output().buildExtractValue(mul_result, 1));
        result = impl().WordToTagged(output().buildExtractValue(mul_result, 0));
      } break;
      case Token::kBIT_OR:
        result = output().buildOr(impl().TaggedToWord(left),
                                  impl().TaggedToWord(right));
        result = impl().WordToTagged(result);
        break;
      case Token::kBIT_AND:
        result = output().buildAnd(impl().TaggedToWord(left),
                                   impl().TaggedToWord(right));
        result = impl().WordToTagged(result);
        break;
      case Token::kBIT_XOR:
        result = output().buildXor(impl().TaggedToWord(left),
                                   impl().TaggedToWord(right));
        result = impl().WordToTagged(result);
        break;
      case Token::kSHL: {
        LValue right_int = impl().SmiUntag(right);
        LValue overflow = output().buildICmp(
            LLVMIntUGT, right_int,
            output().constIntPtr(compiler::target::kSmiBits));
        diamond.BranchToOtherLeafOnCondition(overflow);
        LValue left_int = impl().TaggedToWord(left);
        result = output().buildShl(left_int, right_int);
        LValue check_for_overflow = output().buildSar(result, right_int);
        diamond.BranchToOtherLeafOnCondition(
            output().buildICmp(LLVMIntEQ, check_for_overflow, result));
        result = impl().WordToTagged(result);
      } break;
      case Token::kSHR: {
        LValue right_int = impl().SmiUntag(right);
        LValue overflow = output().buildICmp(
            LLVMIntUGT, right_int,
            output().constIntPtr(compiler::target::kSmiBits));
        diamond.BranchToOtherLeafOnCondition(overflow);

        LValue left_int = impl().SmiUntag(left);
        result = output().buildSar(left_int, right_int);
        result = impl().SmiTag(result);
      } break;
      default:
        UNREACHABLE();
    }
    return result;
  });
  diamond.BuildRight([&] {
    impl().PushArgument(left);
    impl().PushArgument(right);
    const auto& selector = String::Handle(instr->call()->Selector());
    const auto& arguments_descriptor =
        Array::Handle(ArgumentsDescriptor::NewBoxed(/*type_args_len=*/0,
                                                    /*num_arguments=*/2));
    return impl().GenerateMegamorphicInstanceCall(
        instr, selector, arguments_descriptor, instr->call()->deopt_id(),
        instr->token_pos());
  });
  LValue final_result = diamond.End();
  impl().SetLLVMValue(instr, final_result);
}

void IRTranslator::VisitBinaryInt32Op(BinaryInt32OpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue left = impl().EnsureInt32(impl().GetLLVMValue(instr->left()));
  LValue right = impl().EnsureInt32(impl().GetLLVMValue(instr->right()));
  LValue value;
  switch (instr->op_kind()) {
    case Token::kSHL:
      value = output().buildShl(left, right);
      break;
    case Token::kADD:
      value = output().buildAdd(left, right);
      break;
    case Token::kSUB:
      value = output().buildSub(left, right);
      break;
    case Token::kMUL:
      value = output().buildMul(left, right);
      break;
    case Token::kTRUNCDIV:
      value = output().buildSDiv(left, right);
      break;
    case Token::kBIT_AND:
      value = output().buildAnd(left, right);
      break;
    case Token::kBIT_OR:
      value = output().buildOr(left, right);
      break;
    case Token::kBIT_XOR:
      value = output().buildXor(left, right);
      break;
    case Token::kSHR:
      value = output().buildShr(left, right);
      break;
    default:
      UNREACHABLE();
      break;
  }
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitUnarySmiOp(UnarySmiOpInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitUnaryDoubleOp(UnaryDoubleOpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue v = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(v) == output().repo().doubleType);
  impl().SetLLVMValue(instr, output().buildFNeg(v));
}

void IRTranslator::VisitCheckStackOverflow(CheckStackOverflowInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue stack_pointer =
      output().buildCall(output().repo().stackSaveIntrinsic());
  LValue stack_pointer_int =
      output().buildCast(LLVMPtrToInt, stack_pointer, output().repo().intPtr);

  LValue stack_limit = output().buildLoad(output().buildGEPWithByteOffset(
      output().thread(),
      output().constIntPtr(compiler::target::Thread::stack_limit_offset()),
      pointerType(output().repo().intPtr)));
  AssemblerResolver resolver(impl());
  Label slow_path("CheckStackOverflowSlowPath");
  LValue cmp = output().buildICmp(LLVMIntULE, stack_pointer_int, stack_limit);
  resolver.BranchIf(impl().ExpectFalse(cmp), slow_path);
  resolver.GotoMerge();
  resolver.Bind(slow_path);
  if (!instr->UseSharedSlowPathStub(true)) {
    impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                               kStackOverflowRuntimeEntry, 0, false);
  } else if (!impl()
                  .object_store()
                  ->stack_overflow_stub_without_fpu_regs_stub()
                  ->InVMIsolateHeap()) {
    const auto& stub = Code::ZoneHandle(
        impl().zone(),
        impl().object_store()->stack_overflow_stub_without_fpu_regs_stub());
    const auto& fpu_stub = Code::ZoneHandle(
        impl().zone(),
        impl().object_store()->stack_overflow_stub_with_fpu_regs_stub());
    impl().GenerateCall(instr, instr->token_pos(), DeoptId::kNone,
                        impl().GetSharedStubCSR(), stub, &fpu_stub,
                        RawPcDescriptors::kOther, 0, {});
  } else {
    std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
    callsite_info->set_type(CallSiteInfo::CallTargetType::kThreadOffset);
    callsite_info->set_instr_size(kThreadOffsetCallInstrSize);
    callsite_info->set_thread_offset(
        compiler::target::Thread::stack_overflow_shared_stub_entry_point_offset(
            false));
    callsite_info->set_fpu_thread_offset(
        compiler::target::Thread::stack_overflow_shared_stub_entry_point_offset(
            true));

    CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
    CallResolver call_resolver(impl(), -1, param);
    call_resolver.set_callee_save_register(impl().GetSharedStubCSR());
    call_resolver.BuildCall();
  }
  resolver.GotoMerge();
  resolver.End();
}

void IRTranslator::VisitSmiToDouble(SmiToDoubleInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue v = impl().GetLLVMValue(instr->value());
  LValue num = impl().SmiUntag(v);
  LValue dnum = output().buildCast(LLVMSIToFP, num, output().repo().doubleType);
  impl().SetLLVMValue(instr, dnum);
}

void IRTranslator::VisitInt32ToDouble(Int32ToDoubleInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue num = impl().EnsureInt32(impl().GetLLVMValue(instr->value()));
  LValue dnum = output().buildCast(LLVMSIToFP, num, output().repo().doubleType);
  impl().SetLLVMValue(instr, dnum);
}

void IRTranslator::VisitInt64ToDouble(Int64ToDoubleInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue num = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(num) == output().repo().int64);
  LValue dnum = output().buildCast(LLVMSIToFP, num, output().repo().doubleType);
  impl().SetLLVMValue(instr, dnum);
}

void IRTranslator::VisitDoubleToInteger(DoubleToIntegerInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitDoubleToSmi(DoubleToSmiInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitDoubleToDouble(DoubleToDoubleInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitDoubleToFloat(DoubleToFloatInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue fnum = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(fnum) == output().repo().doubleType);
  LValue dnum =
      output().buildCast(LLVMFPTrunc, fnum, output().repo().floatType);
  impl().SetLLVMValue(instr, dnum);
}

void IRTranslator::VisitFloatToDouble(FloatToDoubleInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue fnum = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(fnum) == output().repo().floatType);
  LValue dnum = output().buildCast(LLVMFPExt, fnum, output().repo().doubleType);
  impl().SetLLVMValue(instr, dnum);
}

void IRTranslator::VisitCheckClass(CheckClassInstr* instr) {
  impl().set_exception_occured();
}

void IRTranslator::VisitCheckClassId(CheckClassIdInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitCheckSmi(CheckSmiInstr* instr) {
  // exception
  impl().set_exception_occured();
}

void IRTranslator::VisitCheckNull(CheckNullInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  AssemblerResolver resolver(impl());
  Label slow_path("CheckNullSlowPath");
  LValue null_object = impl().GetNull();
  LValue cmp = output().buildICmp(LLVMIntEQ, null_object, value);
  resolver.BranchIf(impl().ExpectFalse(cmp), slow_path);
  resolver.GotoMerge();
  resolver.Bind(slow_path);
  const RuntimeEntry* runtime_entry;
  if (instr->IsArgumentCheck()) {
    runtime_entry = &kArgumentNullErrorRuntimeEntry;
  } else {
    runtime_entry = &kNullErrorRuntimeEntry;
  }
  impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                             *runtime_entry, 0, false);
  output().buildCall(output().repo().trapIntrinsic());
  resolver.GotoMerge();
  resolver.End();
}

void IRTranslator::VisitCheckCondition(CheckConditionInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitConstant(ConstantInstr* instr) {
  impl().SetLazyValue(instr);
}

void IRTranslator::VisitUnboxedConstant(UnboxedConstantInstr* instr) {
  impl().SetLazyValue(instr);
}

void IRTranslator::VisitCheckEitherNonSmi(CheckEitherNonSmiInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitBinaryDoubleOp(BinaryDoubleOpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue left = impl().GetLLVMValue(instr->left());
  LValue right = impl().GetLLVMValue(instr->right());
  EMASSERT(typeOf(left) == output().repo().doubleType);
  EMASSERT(typeOf(right) == output().repo().doubleType);
  LValue value;
  switch (instr->op_kind()) {
    case Token::kADD:
      value = output().buildFAdd(left, right);
      break;
    case Token::kSUB:
      value = output().buildFSub(left, right);
      break;
    case Token::kMUL:
      value = output().buildFMul(left, right);
      break;
    case Token::kDIV:
      value = output().buildFDiv(left, right);
      break;
    default:
      UNREACHABLE();
  }
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitDoubleTestOp(DoubleTestOpInstr* instr) {
  impl().SetCurrentInstr(instr);
  ComparisonResolver resolver(impl());
  instr->Accept(&resolver);
  impl().SetLLVMValue(instr, impl().BooleanToObject(resolver.result()));
}

void IRTranslator::VisitMathUnary(MathUnaryInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(value) == output().repo().doubleType);
  if (instr->kind() == MathUnaryInstr::kSqrt) {
    value = output().buildCall(output().repo().doubleSqrtIntrinsic(), value);
  } else if (instr->kind() == MathUnaryInstr::kDoubleSquare) {
    value = output().buildFMul(value, value);
  } else {
    UNREACHABLE();
  }
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitMathMinMax(MathMinMaxInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue left = impl().GetLLVMValue(instr->left());
  LValue right = impl().GetLLVMValue(instr->right());
  LValue value;
  LValue cmp_value;
  const intptr_t is_min = (instr->op_kind() == MethodRecognizer::kMathMin);
  if (instr->result_cid() == kDoubleCid) {
    EMASSERT(typeOf(left) == output().repo().doubleType);
    EMASSERT(typeOf(right) == output().repo().doubleType);
    cmp_value = output().buildFCmp(LLVMRealOGT, left, right);
  } else {
    cmp_value = output().buildICmp(LLVMIntSGT, left, right);
  }
  if (is_min)
    value = output().buildSelect(cmp_value, right, left);
  else
    value = output().buildSelect(cmp_value, left, right);
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitBox(BoxInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  auto BoxClassFor = [&](Representation rep) -> const Class& {
    switch (rep) {
      case kUnboxedFloat:
        EMASSERT(typeOf(value) == output().repo().floatType);
        return impl().double_class();
      case kUnboxedDouble:
        EMASSERT(typeOf(value) == output().repo().doubleType);
        return impl().double_class();
      case kUnboxedFloat32x4:
        EMASSERT(typeOf(value) == output().repo().float32x4);
        return impl().float32x4_class();
      case kUnboxedFloat64x2:
        EMASSERT(typeOf(value) == output().repo().float64x2);
        return impl().float64x2_class();
      case kUnboxedInt32x4:
        EMASSERT(typeOf(value) == output().repo().int32x4);
        return impl().float32x4_class();
      case kUnboxedInt64:
        EMASSERT(typeOf(value) == output().repo().int64);
        return impl().mint_class();
      default:
        UNREACHABLE();
        return Class::ZoneHandle();
    }
  };

  BoxAllocationSlowPath allocate(
      instr, BoxClassFor(instr->from_representation()), impl());
  LValue result = allocate.Allocate();
  switch (instr->from_representation()) {
    case kUnboxedFloat32x4:
    case kUnboxedFloat64x2:
    case kUnboxedInt32x4:
    case kUnboxedDouble:
      impl().StoreToOffset(result, instr->ValueOffset() - kHeapObjectTag,
                           value);
      break;
    default:
      UNREACHABLE();
      break;
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitUnbox(UnboxInstr* instr) {
  // FIXME: need to review 64 bit integer.
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  LValue result;
  auto EmitLoadFromBox = [&]() {
    switch (instr->representation()) {
      case kUnboxedInt64: {
        LValue gep = output().buildGEPWithByteOffset(
            value,
            Boxing::ValueOffset(instr->representation()) - kHeapObjectTag,
            pointerType(output().repo().int64));
        return output().buildLoad(gep);
      }

      case kUnboxedDouble: {
        LValue gep = output().buildGEPWithByteOffset(
            value,
            Boxing::ValueOffset(instr->representation()) - kHeapObjectTag,
            pointerType(output().repo().doubleType));
        return output().buildLoad(gep);
      }

      case kUnboxedFloat: {
        LValue gep = output().buildGEPWithByteOffset(
            value,
            Boxing::ValueOffset(instr->representation()) - kHeapObjectTag,
            pointerType(output().repo().doubleType));
        LValue result = output().buildLoad(gep);
        return output().buildCast(LLVMFPTrunc, result,
                                  output().repo().floatType);
      }

      case kUnboxedFloat32x4: {
        LValue gep = output().buildGEPWithByteOffset(
            value,
            Boxing::ValueOffset(instr->representation()) - kHeapObjectTag,
            pointerType(output().repo().float32x4));
        return output().buildLoad(gep);
      }
      case kUnboxedFloat64x2: {
        LValue gep = output().buildGEPWithByteOffset(
            value,
            Boxing::ValueOffset(instr->representation()) - kHeapObjectTag,
            pointerType(output().repo().float64x2));
        return output().buildLoad(gep);
      }
      case kUnboxedInt32x4: {
        LValue gep = output().buildGEPWithByteOffset(
            value,
            Boxing::ValueOffset(instr->representation()) - kHeapObjectTag,
            pointerType(output().repo().int32x4));
        return output().buildLoad(gep);
      }

      default:
        UNREACHABLE();
        break;
    }
  };

  auto EmitLoadInt32FromBoxOrSmi = [&]() {
    DiamondContinuationResolver diamond(impl(), instr->ssa_temp_index());
    diamond.BuildCmp([&]() { return impl().IsSmi(value); })
        .BuildLeft([&]() { return impl().SmiUntag(value); })
        .BuildRight([&]() {
          return impl().LoadFieldFromOffset(
              value, compiler::target::Mint::value_offset(),
              output().repo().ref32);
        });
    return diamond.End();
  };
  auto EmitSmiConversion = [&]() {
    LValue result;
    switch (instr->representation()) {
      case kUnboxedInt64: {
        result = impl().SmiUntag(value);
        result = output().buildCast(LLVMSExt, result, output().repo().int64);
        break;
      }

      case kUnboxedDouble: {
        result = impl().SmiUntag(value);
        result =
            output().buildCast(LLVMSIToFP, result, output().repo().doubleType);
        break;
      }

      default:
        UNREACHABLE();
        break;
    }
    return result;
  };
  auto EmitLoadInt64FromBoxOrSmi = [&]() {
    DiamondContinuationResolver diamond(impl(), instr->ssa_temp_index());
    diamond.BuildCmp([&]() { return impl().IsSmi(value); })
        .BuildLeft([&]() {
          LValue result = impl().SmiUntag(value);
          return output().buildCast(LLVMSExt, result, output().repo().int64);
        })
        .BuildRight([&]() { return EmitLoadFromBox(); });
    return diamond.End();
  };
  if (instr->SpeculativeModeOfInputs() == UnboxInstr::kNotSpeculative) {
    switch (instr->representation()) {
      case kUnboxedInt32x4:
      case kUnboxedFloat32x4:
      case kUnboxedFloat64x2:
      case kUnboxedDouble:
      case kUnboxedFloat: {
        result = EmitLoadFromBox();
      } break;

      case kUnboxedInt32:
      case kUnboxedUint32:
        result = EmitLoadInt32FromBoxOrSmi();
        break;

      case kUnboxedInt64: {
        if (instr->value()->Type()->ToCid() == kSmiCid) {
          // Smi -> int64 conversion is more efficient than
          // handling arbitrary smi/mint.
          result = EmitSmiConversion();
        } else {
          result = EmitLoadInt64FromBoxOrSmi();
        }
        break;
      }
      default:
        UNREACHABLE();
        break;
    }
  } else {
    ASSERT(instr->SpeculativeModeOfInputs() == UnboxInstr::kGuardInputs);
    const intptr_t value_cid = instr->value()->Type()->ToCid();
    const intptr_t box_cid = Boxing::BoxCid(instr->representation());

    if (value_cid == box_cid) {
      result = EmitLoadFromBox();
    } else if (instr->CanConvertSmi() && (value_cid == kSmiCid)) {
      result = EmitSmiConversion();
    } else if (instr->representation() == kUnboxedInt32 &&
               instr->value()->Type()->IsInt()) {
      result = EmitLoadInt32FromBoxOrSmi();
    } else if (instr->representation() == kUnboxedInt64 &&
               instr->value()->Type()->IsInt()) {
      result = EmitLoadInt64FromBoxOrSmi();
    } else {
      EMASSERT(instr->CanDeoptimize());
      // this caused by AotCallSpecializer
      result = LLVMGetUndef(
          impl().GetMachineRepresentationType(instr->representation()));
      impl().set_exception_occured();
    }
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitBoxInt64(BoxInt64Instr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  EMASSERT(typeOf(value) == output().repo().int64);
  LValue result;
  if (instr->ValueFitsSmi()) {
    value = output().buildCast(LLVMTrunc, value, output().repo().intPtr);
    result = impl().SmiTag(value);
  } else {
    AssemblerResolver resolver(impl());
    Label slow_path("Allocate for BoxInt64:%d", instr->ssa_temp_index());
#if defined(TARGET_ARCH_IS_32_BIT)
    LValue value_lo =
        output().buildCast(LLVMTrunc, value, output().repo().int32);
    LValue value_hi = output().buildCast(
        LLVMTrunc, output().buildShr(value, output().constInt64(32)),
        output().repo().int32);
    result = output().buildShl(value_lo, output().constInt32(kSmiTagSize));
    LValue cmp_1 = output().buildICmp(
        LLVMIntEQ, output().buildSar(result, output().constInt32(kSmiTagSize)),
        value_lo);
    resolver.BranchIfNot(impl().ExpectTrue(cmp_1), slow_path);
    LValue cmp =
        output().buildICmp(LLVMIntEQ, value_hi,
                           output().buildSar(result, output().constInt32(31)));
#else
    result = output().buildShl(value, output().constInt64(kSmiTagSize));
    LValue cmp = output().buildICmp(
        LLVMIntEQ, value,
        output().buildSar(result, output().constInt64(kSmiTagSize)));
#endif
    resolver.BranchIfNot(impl().ExpectTrue(cmp), slow_path);
    resolver.GotoMergeWithValue(impl().WordToTagged(result));

    resolver.Bind(slow_path);
    if (BoxInt64Instr::SlowPathSharingSupported(true) &&
        FLAG_use_bare_instructions &&
        !impl()
             .object_store()
             ->allocate_mint_with_fpu_regs_stub()
             ->InVMIsolateHeap()) {
      const auto& stub = Code::ZoneHandle(
          impl().zone(),
          impl().object_store()->allocate_mint_without_fpu_regs_stub());
      const auto& fpu_stub = Code::ZoneHandle(
          impl().zone(),
          impl().object_store()->allocate_mint_with_fpu_regs_stub());
      result = impl().GenerateCall(instr, instr->token_pos(), DeoptId::kNone,
                                   impl().GetBoxInt64SharedStubCSR(), stub,
                                   &fpu_stub, RawPcDescriptors::kOther, 0, {});
    } else {
      BoxAllocationSlowPath allocate(instr, impl().mint_class(), impl());
      result = allocate.Allocate();
    }
    impl().StoreToOffset(
        result, compiler::target::Mint::value_offset() - kHeapObjectTag, value);
    resolver.GotoMergeWithValue(result);
    result = resolver.End();
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitUnboxInt64(UnboxInt64Instr* instr) {
  VisitUnbox(instr);
}

void IRTranslator::VisitCaseInsensitiveCompare(
    CaseInsensitiveCompareInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitBinaryInt64Op(BinaryInt64OpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue left = impl().GetLLVMValue(instr->left());
  LValue right = impl().GetLLVMValue(instr->right());
  EMASSERT(typeOf(left) == output().repo().int64);
  EMASSERT(typeOf(right) == output().repo().int64);
  LValue result;
  switch (instr->op_kind()) {
    case Token::kBIT_AND: {
      result = output().buildAnd(left, right);
      break;
    }
    case Token::kBIT_OR: {
      result = output().buildOr(left, right);
      break;
    }
    case Token::kBIT_XOR: {
      result = output().buildXor(left, right);
      break;
    }
    case Token::kADD: {
      result = output().buildAdd(left, right);
      break;
    }
    case Token::kSUB: {
      result = output().buildSub(left, right);
      break;
    }
    case Token::kMUL: {
      result = output().buildMul(left, right);
      break;
    }
    case Token::kMOD: {
      EMASSERT(impl().support_integer_div());
      result = output().buildSRem(left, right);
      break;
    }
    case Token::kTRUNCDIV: {
      EMASSERT(impl().support_integer_div());
      result = output().buildSDiv(left, right);
      break;
    }
    default:
      UNREACHABLE();
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitShiftInt64Op(ShiftInt64OpInstr* instr) {
  impl().SetCurrentInstr(instr);

  LValue left = impl().GetLLVMValue(instr->left());
  LValue right = impl().GetLLVMValue(instr->right());
  EMASSERT(typeOf(left) == output().repo().int64);
  EMASSERT(typeOf(right) == output().repo().int64);

  LValue result;

  switch (instr->op_kind()) {
    case Token::kSHR: {
      result = output().buildShr(left, right);
      break;
    }
    case Token::kSHL: {
      result = output().buildShl(left, right);
      break;
    }
    default:
      UNREACHABLE();
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitSpeculativeShiftInt64Op(
    SpeculativeShiftInt64OpInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitUnaryInt64Op(UnaryInt64OpInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());

  switch (instr->op_kind()) {
    case Token::kBIT_NOT:
      value = output().buildNot(value);
      break;
    case Token::kNEGATE:
      value = output().buildNeg(value);
      break;
    default:
      UNREACHABLE();
  }
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitCheckArrayBound(CheckArrayBoundInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitGenericCheckBound(GenericCheckBoundInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue length = impl().GetLLVMValue(instr->length());
  LValue index = impl().GetLLVMValue(instr->index());
  const intptr_t index_cid = instr->index()->Type()->ToCid();
  AssemblerResolver resolver(impl());
  Label slow_path("GenericCheckBoundInstrSlowPath");
  if (index_cid != kSmiCid) {
    LValue is_smi = impl().IsSmi(index);
    resolver.BranchIfNot(impl().ExpectTrue(is_smi), slow_path);
  }
  LValue cmp = output().buildICmp(LLVMIntUGE, index, length);
  resolver.BranchIf(impl().ExpectFalse(cmp), slow_path);
  resolver.GotoMerge();
  resolver.Bind(slow_path);
  impl().PushArgument(length);
  impl().PushArgument(index);
  impl().GenerateRuntimeCall(instr, instr->token_pos(), instr->deopt_id(),
                             kRangeErrorRuntimeEntry, 2, false);
  output().buildCall(output().repo().trapIntrinsic());
  resolver.GotoMerge();
  resolver.End();
}

void IRTranslator::VisitConstraint(ConstraintInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitStringToCharCode(StringToCharCodeInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitOneByteStringFromCharCode(
    OneByteStringFromCharCodeInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitStringInterpolate(StringInterpolateInstr* instr) {
  impl().SetCurrentInstr(instr);
  const int kTypeArgsLen = 0;
  const int kNumberOfArguments = 1;
  constexpr const int kSizeOfArguments = 1;
  const Array& kNoArgumentNames = Object::null_array();
  ArgumentsInfo args_info(kTypeArgsLen, kNumberOfArguments, kSizeOfArguments,
                          kNoArgumentNames);
  std::vector<LValue> arguments{impl().GetLLVMValue(instr->value())};
  LValue result = impl().GenerateStaticCall(
      instr, instr->ssa_temp_index(), arguments, instr->deopt_id(),
      instr->token_pos(), instr->CallFunction(), args_info, ICData::Handle());
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitInvokeMathCFunction(InvokeMathCFunctionInstr* instr) {
  impl().SetCurrentInstr(instr);
  // init the types & parameters.
  std::vector<LType> operand_value_types;
  std::vector<LValue> parameters;
  for (intptr_t i = 0; i < instr->InputCount(); ++i) {
    operand_value_types.emplace_back(impl().GetMachineRepresentationType(
        instr->RequiredInputRepresentation(i)));
    parameters.emplace_back(impl().GetLLVMValue(instr->InputAt(i)));
  }
  LType return_type =
      impl().GetMachineRepresentationType(instr->representation());
  LType callee_function_type =
      functionType(return_type, operand_value_types.data(),
                   operand_value_types.size(), NotVariadic);
  LType callee_type = pointerType(callee_function_type);
  // get the statepoint function.
  LValue statepoint_function = output().getStatePointFunction(callee_type);
  // init statepoint operands.
  int64_t patchid = impl().NextPatchPoint();
  std::vector<LValue> statepoint_operands;
  statepoint_operands.push_back(output().constInt64(patchid));
  statepoint_operands.push_back(
      output().constInt32(kInvokeCFunctionCallInstrSize));
  statepoint_operands.push_back(LLVMGetUndef(callee_type));

  statepoint_operands.push_back(
      output().constInt32(parameters.size()));            // # call params
  statepoint_operands.push_back(output().constInt32(0));  // flags
  for (LValue parameter : parameters)
    statepoint_operands.push_back(parameter);
  statepoint_operands.push_back(output().constInt32(0));  // # transition args
  statepoint_operands.push_back(output().constInt32(0));  // # deopt arguments
  // call the function.
  LValue statepoint_value =
      output().buildCall(statepoint_function, statepoint_operands.data(),
                         statepoint_operands.size());
  // setup attribute.
  static const char kDartCCall[] = "dart-c-call";
  LLVMAttributeRef attr =
      output().createStringAttr(kDartCCall, sizeof(kDartCCall) - 1, nullptr, 0);
  LLVMAddCallSiteAttribute(statepoint_value, ~0U, attr);
  if (impl().GetCCallCSR())
    output().assignCSRAttr(statepoint_value, impl().GetCCallCSR());

  LValue intrinsic = output().getGCResultFunction(return_type);
  LValue call_value = output().buildCall(intrinsic, statepoint_value);
  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo());
  callsite_info->set_patchid(patchid);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kCCall);
  callsite_info->set_instr_size(kInvokeCFunctionCallInstrSize);
  callsite_info->set_runtime_entry(&instr->TargetFunction());
  callsite_info->set_stack_parameter_count(instr->InputCount());
  impl().SubmitStackMap(std::move(callsite_info));

  impl().SetLLVMValue(instr, call_value);
}

void IRTranslator::VisitTruncDivMod(TruncDivModInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitGuardFieldClass(GuardFieldClassInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitGuardFieldLength(GuardFieldLengthInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitGuardFieldType(GuardFieldTypeInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

static bool IsPowerOfTwoKind(intptr_t v1, intptr_t v2) {
  return (Utils::IsPowerOfTwo(v1) && (v2 == 0)) ||
         (Utils::IsPowerOfTwo(v2) && (v1 == 0));
}

void IRTranslator::VisitIfThenElse(IfThenElseInstr* instr) {
  impl().SetCurrentInstr(instr);
  ComparisonResolver resolver(impl());
  instr->comparison()->Accept(&resolver);
  LValue cmp_value = resolver.result();

  intptr_t true_value = instr->if_true();
  intptr_t false_value = instr->if_false();

  const bool is_power_of_two_kind = IsPowerOfTwoKind(true_value, false_value);

  if (is_power_of_two_kind) {
    if (true_value == 0) {
      // We need to have zero in result on true_condition.
      cmp_value = output().buildICmp(LLVMIntEQ, cmp_value,
                                     output().repo().booleanFalse);
    }
  } else {
    if (true_value == 0) {
      // Swap values so that false_value is zero.
      intptr_t temp = true_value;
      true_value = false_value;
      false_value = temp;
    } else {
      cmp_value = output().buildICmp(LLVMIntEQ, cmp_value,
                                     output().repo().booleanFalse);
    }
  }
  LValue result = output().buildSelect(cmp_value, output().constIntPtr(1),
                                       output().constIntPtr(0));

  if (is_power_of_two_kind) {
    const intptr_t shift =
        Utils::ShiftForPowerOfTwo(Utils::Maximum(true_value, false_value));
    result =
        output().buildShl(result, output().constIntPtr(shift + kSmiTagSize));
  } else {
    result = output().buildSub(result, output().constIntPtr(1));
    const int32_t val = compiler::target::ToRawSmi(true_value) -
                        compiler::target::ToRawSmi(false_value);
    result = output().buildAnd(result, output().constIntPtr(val));
    if (false_value != 0) {
      result = output().buildAdd(
          result,
          output().constIntPtr(compiler::target::ToRawSmi(false_value)));
    }
  }
  result = impl().WordToTagged(result);
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitMaterializeObject(MaterializeObjectInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitTestSmi(TestSmiInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitTestCids(TestCidsInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitExtractNthOutput(ExtractNthOutputInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitBinaryUint32Op(BinaryUint32OpInstr* instr) {
  impl().SetCurrentInstr(instr);

  LValue left = impl().GetLLVMValue(instr->left());
  LValue right = impl().GetLLVMValue(instr->right());
  EMASSERT(typeOf(left) == output().repo().int32);
  EMASSERT(typeOf(right) == output().repo().int32);
  LValue result;
  switch (instr->op_kind()) {
    case Token::kBIT_AND:
      result = output().buildAnd(left, right);
      break;
    case Token::kBIT_OR:
      result = output().buildOr(left, right);
      break;
    case Token::kBIT_XOR:
      result = output().buildXor(left, right);
      break;
    case Token::kADD:
      result = output().buildAdd(left, right);
      break;
    case Token::kSUB:
      result = output().buildSub(left, right);
      break;
    case Token::kMUL:
      result = output().buildMul(left, right);
      break;
    default:
      UNREACHABLE();
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitShiftUint32Op(ShiftUint32OpInstr* instr) {
  impl().SetCurrentInstr(instr);

  LValue left = impl().GetLLVMValue(instr->left());
  LValue right = impl().GetLLVMValue(instr->right());

  LValue result;
  // FIXME: Check right < 0 then goto slow path(throw).
  if (typeOf(right) == output().repo().int64)
    right = output().buildCast(LLVMTrunc, right, output().repo().int32);

  switch (instr->op_kind()) {
    case Token::kSHR: {
      result = output().buildShr(left, right);
      break;
    }
    case Token::kSHL: {
      result = output().buildShl(left, right);
      break;
    }
    default:
      UNREACHABLE();
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitSpeculativeShiftUint32Op(
    SpeculativeShiftUint32OpInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitUnaryUint32Op(UnaryUint32OpInstr* instr) {
  impl().SetCurrentInstr(instr);
  EMASSERT(instr->op_kind() == Token::kBIT_NOT);
  LValue value = impl().GetLLVMValue(instr->value());

  value = output().buildNot(value);
  impl().SetLLVMValue(instr, value);
}

void IRTranslator::VisitBoxUint32(BoxUint32Instr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  LValue result = impl().BoxInteger32(instr, value, instr->ValueFitsSmi(),
                                      instr->from_representation());
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitUnboxUint32(UnboxUint32Instr* instr) {
  VisitUnboxInteger32(instr);
}

void IRTranslator::VisitBoxInt32(BoxInt32Instr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  LValue result = impl().BoxInteger32(instr, value, instr->ValueFitsSmi(),
                                      instr->from_representation());
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitUnboxInt32(UnboxInt32Instr* instr) {
  VisitUnboxInteger32(instr);
}

void IRTranslator::VisitIntConverter(IntConverterInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  LValue result;
#if defined(TARGET_ARCH_IS_32_BIT)
  const bool is_nop_conversion =
      (instr->from() == kUntagged && instr->to() == kUnboxedInt32) ||
      (instr->from() == kUntagged && instr->to() == kUnboxedUint32) ||
      (instr->from() == kUnboxedInt32 && instr->to() == kUntagged) ||
      (instr->from() == kUnboxedUint32 && instr->to() == kUntagged);
#elif defined(TARGET_ARCH_IS_64_BIT)
  const bool is_nop_conversion =
      (instr->from() == kUntagged && instr->to() == kUnboxedIntPtr) ||
      (instr->from() == kUnboxedIntPtr && instr->to() == kUntagged);
#else
#error unsupported arch
#endif

  if (is_nop_conversion) {
    result = value;
  } else if (instr->from() == kUnboxedInt32 && instr->to() == kUnboxedUint32) {
    result = value;
  } else if (instr->from() == kUnboxedUint32 && instr->to() == kUnboxedInt32) {
    result = value;
  } else if (instr->from() == kUnboxedInt64) {
    if (instr->to() == kUnboxedInt32 || instr->to() == kUnboxedUint32) {
      result = output().buildCast(LLVMTrunc, value, output().repo().int32);
    } else {
      EMASSERT(instr->to() == kUnboxedInt64);
      result = value;
    }
  } else if (instr->to() == kUnboxedInt64) {
    if (instr->from() == kUnboxedUint32) {
      result = output().buildCast(LLVMZExt, value, output().repo().int64);
    } else {
      EMASSERT(instr->from() == kUnboxedInt32);
      result = output().buildCast(LLVMSExt, value, output().repo().int64);
    }
  } else {
    UNREACHABLE();
  }
  impl().SetLLVMValue(instr, result);
}

void IRTranslator::VisitBitCast(BitCastInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitDeoptimize(DeoptimizeInstr* instr) {
  LLVMLOGE("unsupported IR: %s\n", __FUNCTION__);
  UNREACHABLE();
}

void IRTranslator::VisitSimdOp(SimdOpInstr* instr) {
  // AOT will not handle it.
  impl().set_exception_occured();
  impl().SetLLVMValue(instr, LLVMGetUndef(impl().GetMachineRepresentationType(
                                 instr->representation())));
}

void IRTranslator::VisitReachabilityFence(ReachabilityFenceInstr*) {}

void IRTranslator::VisitDispatchTableCall(DispatchTableCallInstr* instr) {
  impl().SetCurrentInstr(instr);
  LValue dispatch_table = impl().GetDispatchTable();
  intptr_t offset =
      (instr->selector()->offset - DispatchTable::OriginElement()) *
      compiler::target::kWordSize;
  LValue cid = impl().GetLLVMValue(instr->class_id());
  cid = output().buildCast(LLVMZExt, cid, output().repo().intPtr);
  LValue offset_val = output().buildAdd(
      output().constIntPtr(offset),
      output().buildShl(cid,
                        output().constIntPtr(compiler::target::kWordSizeLog2)));
  LValue entry_gep = output().buildGEPWithByteOffset(
      dispatch_table, offset_val, pointerType(output().repo().ref8));
  LValue entry = output().buildInvariantLoad(entry_gep);

  int argument_count = instr->ArgumentCount();
  std::unique_ptr<CallSiteInfo> callsite_info(new CallSiteInfo);
  callsite_info->set_type(CallSiteInfo::CallTargetType::kReg);
  callsite_info->set_token_pos(instr->token_pos());
  callsite_info->set_deopt_id(instr->deopt_id());
  callsite_info->set_stack_parameter_count(argument_count);
  callsite_info->set_instr_size(kCallInstrSize);
  CallResolver::CallResolverParameter param(instr, std::move(callsite_info));
  CallResolver resolver(impl(), instr->ssa_temp_index(), param);
  resolver.SetCallTarget(entry);
  for (intptr_t i = argument_count - 1; i >= 0; --i) {
    LValue param = impl().GetLLVMValue(instr->ArgumentValueAt(i));
    resolver.AddStackParameter(param);
  }
  Array& arguments_descriptor = Array::ZoneHandle();
  if (instr->selector()->requires_args_descriptor) {
    ArgumentsInfo args_info(instr->type_args_len(), instr->ArgumentCount(),
                            instr->ArgumentsSize(), instr->argument_names());
    arguments_descriptor = args_info.ToArgumentsDescriptor();
    LValue argument_desc_val = impl().LoadObject(arguments_descriptor);
    resolver.SetGParameter(Output::RegisterToParameterLoc(ARGS_DESC_REG),
                           argument_desc_val);
  }
  LValue v = resolver.BuildCall();
  impl().SetLLVMValue(instr, v);
}

void IRTranslator::VisitUnboxInteger32(UnboxInteger32Instr* instr) {
  impl().SetCurrentInstr(instr);
  LValue value = impl().GetLLVMValue(instr->value());
  const intptr_t value_cid = instr->value()->Type()->ToCid();
  LValue result;
  if (value_cid == kSmiCid) {
    result = impl().SmiUntag(value);
#if !defined(TARGET_ARCH_IS_32_BIT)
    result = output().buildCast(LLVMTrunc, result, output().repo().int32);
#endif
  } else if (value_cid == kMintCid) {
    result = impl().LoadFieldFromOffset(
        value, compiler::target::Mint::value_offset(), output().repo().ref32);
  } else if (!instr->CanDeoptimize()) {
    AssemblerResolver resolver(impl());
    Label is_mint("is_mint");
    resolver.BranchIfNot(impl().ExpectTrue(impl().IsSmi(value)), is_mint);
    LValue smi_value = impl().SmiUntag(value);
#if !defined(TARGET_ARCH_IS_32_BIT)
    smi_value = output().buildCast(LLVMTrunc, smi_value, output().repo().int32);
#endif
    resolver.GotoMergeWithValue(smi_value);
    resolver.Bind(is_mint);
    resolver.GotoMergeWithValue(impl().LoadFieldFromOffset(
        value, compiler::target::Mint::value_offset(), output().repo().ref32));
    result = resolver.End();
  } else {
    result = LLVMGetUndef(output().repo().int32);
    impl().set_exception_occured();
  }
  impl().SetLLVMValue(instr, result);
}
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
