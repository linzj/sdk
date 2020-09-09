// Copyright 2019 UCWeb Co., Ltd.

#include "vm/compiler/backend/llvm/output.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#define FEATURE_DEBUG_INFO 1
#include <llvm-c/DebugInfo.h>

#include "vm/compiler/backend/llvm/compiler_state.h"
#include "vm/compiler/backend/llvm/llvm_log.h"
#include "vm/compiler/backend/llvm/target_specific.h"
#include "vm/constants.h"

namespace dart {
namespace dart_llvm {
Output::Output(CompilerState& state)
    : state_(state),
      repo_(state.context_, state.module_),
      builder_(nullptr),
      di_builder_(nullptr),
      prologue_(nullptr),
      thread_(nullptr),
      args_desc_(nullptr),
      pp_(nullptr),
      bitcast_space_(nullptr),
      return_type_(nullptr),
      subprogram_(nullptr),
      stack_parameter_count_(0) {}

Output::~Output() {
  LLVMDisposeBuilder(builder_);
  LLVMDisposeDIBuilder(di_builder_);
}

// parameter layout: register parameter, stack parameter, float point parameter
void Output::initializeBuild(const RegisterParameterDesc& registerParameters,
                             LType return_type) {
  EMASSERT(!builder_);
  EMASSERT(!prologue_);
  builder_ = LLVMCreateBuilderInContext(state_.context_);
  di_builder_ = LLVMCreateDIBuilderDisallowUnresolved(state_.module_);
  return_type_ = return_type;
  initializeFunction(registerParameters, return_type);
  // FIXME: Add V8 to LLVM.
  LLVMSetGC(state_.function_, "coreclr");

  prologue_ = appendBasicBlock("Prologue");
  positionToBBEnd(prologue_);
  thread_ = LLVMGetParam(state_.function_, RegisterToParameterLoc(THR));
  pp_ = LLVMGetParam(state_.function_, RegisterToParameterLoc(PP));
  args_desc_ =
      LLVMGetParam(state_.function_, RegisterToParameterLoc(ARGS_DESC_REG));

  enum class LateParameterType { Stack, FloatPoint };
  // type, position in stack/double registers, position in parameters;
  std::vector<std::tuple<LateParameterType, int, int>> late_parameters;
  for (auto& registerParameter : registerParameters) {
    if ((registerParameter.type == repo().doubleType) ||
        (registerParameter.type == repo().floatType)) {
      parameters_.emplace_back(nullptr);
      int name = registerParameter.name;
#if V8_TARGET_ARCH_ARM
      // ARM use S0, S1...;
      if (registerParameter.type == repo().floatType) name /= 2;
#endif
      late_parameters.emplace_back(LateParameterType::FloatPoint, name,
                                   parameters_.size() - 1);
    } else if ((registerParameter.name >= 0)) {
      EMASSERT(registerParameter.name < kV8CCRegisterParameterCount);
      LValue rvalue = LLVMGetParam(state_.function_, registerParameter.name);
      parameters_.emplace_back(rvalue);
    } else {
      // callee frame
      stack_parameter_count_++;
      parameters_.emplace_back(nullptr);
      late_parameters.emplace_back(LateParameterType::Stack,
                                   -registerParameter.name - 1,
                                   parameters_.size() - 1);
    }
  }
  for (auto i = late_parameters.begin(); i != late_parameters.end(); ++i) {
    LValue v;
    auto& tuple = *i;
    switch (std::get<0>(tuple)) {
      case LateParameterType::Stack:
        v = LLVMGetParam(state_.function_,
                         kV8CCRegisterParameterCount + std::get<1>(tuple));
        break;
      case LateParameterType::FloatPoint:
        v = LLVMGetParam(state_.function_, kV8CCRegisterParameterCount +
                                               stack_parameter_count_ +
                                               std::get<1>(tuple));
        break;
    }
    parameters_[std::get<2>(tuple)] = v;
  }
  int v_null_index = 0;
  for (LValue v : parameters_) {
    EMASSERT(v != nullptr);
    v_null_index++;
  }
  bitcast_space_ = buildAlloca(arrayType(repo().int8, 16));
}

void Output::initializeFunction(const RegisterParameterDesc& registerParameters,
                                LType return_type) {
  std::vector<LType> params_types;
  params_types.resize(kV8CCRegisterParameterCount, tagged_type());
  params_types[RegisterToParameterLoc(PP)] = tagged_type();
  params_types[RegisterToParameterLoc(THR)] = repo().ref8;
#if defined(TARGET_SUPPORT_DISPATCH_TABLE_REG)
  params_types[RegisterToParameterLoc(DISPATCH_TABLE_REG)] = repo().ref8;
#endif
#if defined(TARGET_SUPPORT_NULL_OBJECT_REG)
  params_types[RegisterToParameterLoc(NULL_REG)] = tagged_type();
#endif
#if defined(TARGET_SUPPORT_BARRIER_MASK_REG)
  params_types[RegisterToParameterLoc(BARRIER_MASK)] = repo().intPtr;
#endif
  EMASSERT(params_types.size() == kV8CCRegisterParameterCount);
  std::vector<LType> float_point_parameter_types;
  LType double_type = repo().doubleType;
  LType float_type = repo().floatType;
  for (auto& registerParameter : registerParameters) {
    if ((registerParameter.type == double_type) ||
        (registerParameter.type == float_type)) {
      int name = registerParameter.name;

#if V8_TARGET_ARCH_ARM
      // ARM use S0, S1...;
      if (registerParameter.type == float_type) name /= 2;
#endif
      EMASSERT(float_point_parameter_types.size() <= static_cast<size_t>(name));
      // FIXME: could be architecture dependent.
      if (float_point_parameter_types.size() < static_cast<size_t>(name)) {
        float_point_parameter_types.resize(static_cast<size_t>(name),
                                           float_type);
      }
      float_point_parameter_types.emplace_back(registerParameter.type);
    } else if (registerParameter.name >= 0) {
      EMASSERT(registerParameter.name < 10);
      params_types[registerParameter.name] = registerParameter.type;
    } else {
      int slot = -registerParameter.name;
      if (params_types.size() <
          static_cast<size_t>(slot + kV8CCRegisterParameterCount))
        params_types.resize(slot + kV8CCRegisterParameterCount, tagged_type());
      params_types[slot - 1 + kV8CCRegisterParameterCount] =
          registerParameter.type;
    }
  }
  EMASSERT(float_point_parameter_types.size() <= 8);
  params_types.insert(
      params_types.end(),
      std::make_move_iterator(float_point_parameter_types.begin()),
      std::make_move_iterator(float_point_parameter_types.end()));
  state_.function_ =
      addFunction(state_.function_name_.c_str(),
                  functionType(return_type, params_types.data(),
                               params_types.size(), NotVariadic));
  setFunctionCallingConv(state_.function_, LLVMV8CallConv);

  // Setup function's frame.
  {
    static const char kDartCall[] = "dart-call";
    LLVMAddTargetDependentFunctionAttr(state_.function_, kDartCall, nullptr);
  }

  char file_name[256];
  int file_name_count =
      snprintf(file_name, 256, "%s.c", state_.function_name_.c_str());
  LLVMMetadataRef file_name_meta = LLVMDIBuilderCreateFile(
      di_builder_, file_name, file_name_count, nullptr, 0);
  LLVMMetadataRef cu = LLVMDIBuilderCreateCompileUnit(
      di_builder_, LLVMDWARFSourceLanguageC, file_name_meta, nullptr, 0, true,
      nullptr, 0, 1, nullptr, 0, LLVMDWARFEmissionLineTablesOnly, 0, false,
      false);
  LLVMMetadataRef subroutine_type = LLVMDIBuilderCreateSubroutineType(
      di_builder_, file_name_meta, nullptr, 0, LLVMDIFlagZero);
  subprogram_ = LLVMDIBuilderCreateFunction(
      di_builder_, cu, state_.function_name_.c_str(),
      strlen(state_.function_name_.c_str()), nullptr, 0, file_name_meta, 1,
      subroutine_type, false, true, 1, LLVMDIFlagZero, true);
  LLVMSetSubprogram(state_.function_, subprogram_);
}

LBasicBlock Output::appendBasicBlock(const char* name) {
  return dart_llvm::appendBasicBlock(state_.context_, state_.function_, name);
}

LBasicBlock Output::appendBasicBlock(LValue function, const char* name) {
  return dart_llvm::appendBasicBlock(state_.context_, function, name);
}

LBasicBlock Output::getInsertionBlock() {
  return LLVMGetInsertBlock(builder_);
}

void Output::positionToBBEnd(LBasicBlock bb) {
  LLVMPositionBuilderAtEnd(builder_, bb);
}

void Output::positionBefore(LValue value) {
  LLVMPositionBuilderBefore(builder_, value);
}

LValue Output::constInt8(int i) {
  return dart_llvm::constInt(repo_.int8, i);
}

LValue Output::constInt16(int i) {
  return dart_llvm::constInt(repo_.int16, i);
}

LValue Output::constInt32(int i) {
  return dart_llvm::constInt(repo_.int32, i);
}

LValue Output::constInt64(long long l) {
  return dart_llvm::constInt(repo_.int64, l);
}

LValue Output::constIntPtr(intptr_t i) {
  return dart_llvm::constInt(repo_.intPtr, i);
}

LValue Output::constDouble(double d) {
  return dart_llvm::constReal(repo_.doubleType, d);
}

LValue Output::constTagged(uintptr_t magic) {
  LValue intptr = constIntPtr(static_cast<intptr_t>(magic));
  return buildCast(LLVMIntToPtr, intptr, tagged_type());
}

LValue Output::constString(const char* s) {
  return LLVMConstStringInContext(state_.context_, s, strlen(s), false);
}

LValue Output::buildStructGEP(LValue structVal, unsigned field) {
  return setInstrDebugLoc(
      dart_llvm::buildStructGEP(builder_, structVal, field));
}

LValue Output::buildLoad(LValue toLoad) {
  return setInstrDebugLoc(dart_llvm::buildLoad(builder_, toLoad));
}

LValue Output::buildInvariantLoad(LValue toLoad) {
  LValue load = dart_llvm::buildLoad(builder_, toLoad);
  LValue mdnode = LLVMMDNodeInContext(state_.context_, nullptr, 0);
  constexpr static const unsigned kMD_invariant_load = 6;
  LLVMSetMetadata(load, kMD_invariant_load, mdnode);
  return setInstrDebugLoc(load);
}

LValue Output::buildLoadUnaligned(LValue toLoad) {
  LValue memcpy_function = repo().memcpyIntrinsic();
  LType pointer_type = typeOf(toLoad);
  LType element_type = LLVMGetElementType(pointer_type);
  LValue size_of_element_type = LLVMSizeOf(element_type);
  buildCall(memcpy_function, buildBitCast(bitcast_space_, repo().ref8),
            buildBitCast(toLoad, repo().ref8),
            buildCast(LLVMTrunc, size_of_element_type, repo().int32),
            repo().booleanFalse);
  return buildLoad(buildBitCast(bitcast_space_, pointer_type));
}

LValue Output::buildLoadSmi(LValue toLoad) {
  LValue load = dart_llvm::buildLoad(builder_, toLoad);
  assignEvenNumberAttr(load);
  return setInstrDebugLoc(load);
}

LValue Output::buildStore(LValue val, LValue pointer) {
  return setInstrDebugLoc(dart_llvm::buildStore(builder_, val, pointer));
}

LValue Output::buildNeg(LValue val) {
  return setInstrDebugLoc(dart_llvm::buildNeg(builder_, val));
}

LValue Output::buildAdd(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildAdd(builder_, lhs, rhs));
}

LValue Output::buildFAdd(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildFAdd(builder_, lhs, rhs));
}

LValue Output::buildNSWAdd(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildNSWAdd(builder_, lhs, rhs, ""));
}

LValue Output::buildSub(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildSub(builder_, lhs, rhs));
}

LValue Output::buildFSub(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildFSub(builder_, lhs, rhs));
}

LValue Output::buildNSWSub(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildNSWSub(builder_, lhs, rhs, ""));
}

LValue Output::buildSRem(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildSRem(builder_, lhs, rhs, ""));
}

LValue Output::buildSDiv(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildSDiv(builder_, lhs, rhs, ""));
}

LValue Output::buildFMul(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildFMul(builder_, lhs, rhs));
}

LValue Output::buildFDiv(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildFDiv(builder_, lhs, rhs));
}

LValue Output::buildFCmp(LRealPredicate cond, LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildFCmp(builder_, cond, lhs, rhs));
}

LValue Output::buildFNeg(LValue value) {
  return setInstrDebugLoc(LLVMBuildFNeg(builder_, value, ""));
}

LValue Output::buildNSWMul(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildNSWMul(builder_, lhs, rhs, ""));
}

LValue Output::buildMul(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildMul(builder_, lhs, rhs, ""));
}

LValue Output::buildShl(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildShl(builder_, lhs, rhs));
}

LValue Output::buildShr(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildLShr(builder_, lhs, rhs));
}

LValue Output::buildSar(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildAShr(builder_, lhs, rhs));
}

LValue Output::buildAnd(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildAnd(builder_, lhs, rhs));
}

LValue Output::buildOr(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(dart_llvm::buildOr(builder_, lhs, rhs));
}

LValue Output::buildXor(LValue lhs, LValue rhs) {
  return setInstrDebugLoc(LLVMBuildXor(builder_, lhs, rhs, ""));
}

LValue Output::buildNot(LValue v) {
  return setInstrDebugLoc(LLVMBuildNot(builder_, v, ""));
}

LValue Output::buildBr(LBasicBlock bb) {
  return setInstrDebugLoc(dart_llvm::buildBr(builder_, bb));
}

LValue Output::buildSwitch(LValue val,
                           LBasicBlock defaultBlock,
                           unsigned cases) {
  return setInstrDebugLoc(LLVMBuildSwitch(builder_, val, defaultBlock, cases));
}

LValue Output::buildCondBr(LValue condition,
                           LBasicBlock taken,
                           LBasicBlock notTaken) {
  return setInstrDebugLoc(
      dart_llvm::buildCondBr(builder_, condition, taken, notTaken));
}

LValue Output::buildRet(LValue ret) {
  return setInstrDebugLoc(dart_llvm::buildRet(builder_, ret));
}

LValue Output::buildRetVoid(void) {
  return setInstrDebugLoc(dart_llvm::buildRetVoid(builder_));
}

void Output::buildReturnForTailCall() {
  buildRet(LLVMGetUndef(
      LLVMGetReturnType(LLVMGetElementType(typeOf(state_.function_)))));
}

void Output::buildRetUndef() {
  dart_llvm::buildRet(builder_, LLVMGetUndef(return_type_));
}

LValue Output::buildCast(LLVMOpcode Op, LLVMValueRef Val, LLVMTypeRef DestTy) {
  return setInstrDebugLoc(LLVMBuildCast(builder_, Op, Val, DestTy, ""));
}

LValue Output::buildPointerCast(LValue val, LType type) {
  return setInstrDebugLoc(LLVMBuildPointerCast(builder_, val, type, ""));
}

LValue Output::buildSelect(LValue condition, LValue taken, LValue notTaken) {
  return setInstrDebugLoc(
      dart_llvm::buildSelect(builder_, condition, taken, notTaken));
}

LValue Output::buildICmp(LIntPredicate cond, LValue left, LValue right) {
  return setInstrDebugLoc(dart_llvm::buildICmp(builder_, cond, left, right));
}

LValue Output::buildInlineAsm(LType type,
                              char* asmString,
                              size_t asmStringSize,
                              char* constraintString,
                              size_t constraintStringSize,
                              bool sideEffect) {
  LValue func = LLVMGetInlineAsm(type, asmString, asmStringSize,
                                 constraintString, constraintStringSize,
                                 sideEffect, false, LLVMInlineAsmDialectATT);
  return buildCall(func);
}

LValue Output::buildPhi(LType type) {
  return setInstrDebugLoc(dart_llvm::buildPhi(builder_, type));
}

LValue Output::buildAlloca(LType type) {
  return setInstrDebugLoc(dart_llvm::buildAlloca(builder_, type));
}

LValue Output::buildGEPWithByteOffset(LValue base,
                                      LValue offset,
                                      LType dstType) {
  LType base_type = typeOf(base);
  unsigned base_type_address_space = LLVMGetPointerAddressSpace(base_type);
  unsigned dst_type_address_space = LLVMGetPointerAddressSpace(dstType);
  LValue base_ref8 =
      buildBitCast(base, LLVMPointerType(repo().int8, base_type_address_space));
  LValue offset_value = offset;
  LValue dst_ref8 = LLVMBuildGEP(builder_, base_ref8, &offset_value, 1, "");
  if (base_type_address_space != dst_type_address_space) {
    dst_ref8 = buildCast(LLVMAddrSpaceCast, dst_ref8,
                         LLVMPointerType(repo().int8, dst_type_address_space));
  }
  return buildBitCast(dst_ref8, dstType);
}

LValue Output::buildGEPWithByteOffset(LValue base, int offset, LType dstType) {
  return buildGEPWithByteOffset(base, constInt32(offset), dstType);
}

LValue Output::buildGEP(LValue base, LValue offset) {
  LValue dst = LLVMBuildGEP(builder_, base, &offset, 1, "");
  return dst;
}

LValue Output::buildBitCast(LValue val, LType type) {
  return dart_llvm::buildBitCast(builder_, val, type);
}

void Output::buildUnreachable() {
  setInstrDebugLoc(dart_llvm::buildUnreachable(builder_));
}

static std::string GetMangledTypeStr(LType* types, size_t ntypes) {
  size_t nlength_used;
  // This shit use strdup.
  char* name = const_cast<char*>(
      LLVMIntrinsicCopyOverloadedName(1, types, ntypes, &nlength_used));
  char* last_dot = strrchr(name, '.');
  std::string r(last_dot);
  free(name);
  return r;
}

LValue Output::getStatePointFunction(LType callee_type) {
  auto found = gc_function_map_.find(callee_type);
  if (found != gc_function_map_.end()) return found->second;
  std::vector<LType> wrapped_argument_types;
  wrapped_argument_types.emplace_back(repo().int64);
  wrapped_argument_types.emplace_back(repo().int32);
  wrapped_argument_types.emplace_back(callee_type);
  wrapped_argument_types.emplace_back(repo().int32);
  wrapped_argument_types.emplace_back(repo().int32);
  LType function_type =
      functionType(repo().tokenType, wrapped_argument_types.data(),
                   wrapped_argument_types.size(), Variadic);
  std::string name("llvm.experimental.gc.statepoint");
  name.append(GetMangledTypeStr(&callee_type, 1));
  LValue function =
      addExternFunction(state_.module_, name.c_str(), function_type);
  gc_function_map_[callee_type] = function;
  return function;
}

LValue Output::getGCResultFunction(LType return_type) {
  auto found = gc_function_map_.find(return_type);
  if (found != gc_function_map_.end()) return found->second;
  std::string name("llvm.experimental.gc.result");
  name.append(GetMangledTypeStr(&return_type, 1));
  LType function_type = functionType(return_type, repo().tokenType);
  LValue function =
      addExternFunction(state_.module_, name.c_str(), function_type);
  gc_function_map_[return_type] = function;
  return function;
}

LValue Output::buildExtractValue(LValue aggVal, unsigned index) {
  return dart_llvm::buildExtractValue(builder_, aggVal, index);
}

LValue Output::buildInsertValue(LValue aggVal, unsigned index, LValue value) {
  return LLVMBuildInsertValue(builder_, aggVal, value, index, "");
}

LValue Output::buildCall(LValue function,
                         const LValue* args,
                         unsigned numArgs) {
  return setInstrDebugLoc(LLVMBuildCall(
      builder_, function, const_cast<LValue*>(args), numArgs, ""));
}

LValue Output::buildInvoke(LValue function,
                           const LValue* args,
                           unsigned numArgs,
                           LBasicBlock then,
                           LBasicBlock exception) {
  return setInstrDebugLoc(LLVMBuildInvoke(builder_, function,
                                          const_cast<LValue*>(args), numArgs,
                                          then, exception, ""));
}

LValue Output::buildLandingPad() {
  LValue function = repo().fakePersonalityIntrinsic();
  LType landing_type = repo().tokenType;
  LValue landing_pad =
      LLVMBuildLandingPad(builder_, landing_type, function, 0, "");
  LLVMSetCleanup(landing_pad, true);
  return setInstrDebugLoc(landing_pad);
}

void Output::setDebugInfo(intptr_t linenum, const char* source_file_name) {
#if defined(FEATURE_DEBUG_INFO)
  LLVMMetadataRef scope = subprogram_;
  if (source_file_name) {
    LLVMMetadataRef file_name_meta = LLVMDIBuilderCreateFile(
        di_builder_, source_file_name, strlen(source_file_name), nullptr, 0);
    scope = LLVMDIBuilderCreateLexicalBlockFile(di_builder_, subprogram_,
                                                file_name_meta, 0);
  }
  LLVMMetadataRef loc = LLVMDIBuilderCreateDebugLocation(
      state_.context_, linenum, 1, scope, nullptr);
  LValue loc_value = LLVMMetadataAsValue(state_.context_, loc);
  LLVMSetCurrentDebugLocation(builder_, loc_value);
#endif
}

#if defined(FEATURE_DEBUG_INFO)
static bool ValueKindIsFind(LValue v) {
  switch (LLVMGetValueKind(v)) {
    case LLVMConstantExprValueKind:
    case LLVMConstantIntValueKind:
    case LLVMConstantFPValueKind:
    case LLVMConstantPointerNullValueKind:
    case LLVMArgumentValueKind:
      return false;
    default:
      return true;
  }
}
#endif

LValue Output::setInstrDebugLoc(LValue v) {
#if defined(FEATURE_DEBUG_INFO)
  if (ValueKindIsFind(v)) LLVMSetInstDebugLocation(builder_, v);
#endif
  return v;
}

void Output::assignEvenNumberAttr(LValue load) {
  constexpr static const unsigned kMD_even_number = 30;
  LValue mdnode = LLVMMDNodeInContext(state_.context_, nullptr, 0);
  LLVMSetMetadata(load, kMD_even_number, mdnode);
}

void Output::assignCSRAttr(LValue call, const char* csr) {
  static const char kCustomRegmask[] = "custom-regmask";
  LLVMAttributeRef attr = createStringAttr(
      kCustomRegmask, sizeof(kCustomRegmask) - 1, csr, strlen(csr));
  LLVMAddCallSiteAttribute(call, ~0U, attr);
}

void Output::finalizeDebugInfo() {
  LLVMDIBuilderFinalize(di_builder_);
}

void Output::finalize() {
  finalizeDebugInfo();
}

LValue Output::addFunction(const char* name, LType type) {
  LValue function = dart_llvm::addFunction(state_.module_, name, type);
  AddFunctionCommonAttr(function);
  return function;
}

LType Output::tagged_pair() const {
  return structType(state_.context_, repo_.tagged_type, repo_.tagged_type);
}

void Output::EmitDebugInfo(std::vector<Instruction*>&& debug_instrs) {
  state_.debug_instrs_ = std::move(debug_instrs);
}

void Output::EmitStackMapInfoMap(StackMapInfoMap&& stack_map_info_map) {
  state_.stack_map_info_map_ = std::move(stack_map_info_map);
}

LValue Output::GetRegisterParameter(Register r) {
  int loc = RegisterToParameterLoc(r);
  return LLVMGetParam(state_.function_, loc);
}

LValue Output::fp() {
  return buildCall(repo().frameAddressIntrinsic(), constInt32(0));
}

void Output::AddFunctionCommonAttr(LValue function) {
  // arm jump tables are slow.
#if 0
  static const char kNoJumpTables[] = "no-jump-tables";
  static const char kTrue[] = "true";
  LLVMAddTargetDependentFunctionAttr(function, kNoJumpTables, kTrue);
#endif

  static const char kTrue[] = "true";
  static const char kFalse[] = "false";
  (void)kFalse;
#if defined(TARGET_ARCH_ARM)
  static const char kFS[] = "target-features";
  static const char kFSValue[] =
      "+armv7-a,+dsp,+neon,+vfp3,-crypto,-fp-armv8,-thumb-mode,-vfp4";
  LLVMAddTargetDependentFunctionAttr(function, kFS, kFSValue);
#elif defined(TARGET_ARCH_ARM64)
  static const char kFS[] = "target-features";
  static const char kFSValue[] = "+neon";
  LLVMAddTargetDependentFunctionAttr(function, kFS, kFSValue);
#endif

  static const char kNoRealignStack[] = "no-realign-stack";
  LLVMAddTargetDependentFunctionAttr(function, kNoRealignStack, kTrue);
}

LLVMAttributeRef Output::createStringAttr(const char* key,
                                          unsigned key_len,
                                          const char* value,
                                          unsigned value_len) {
  return LLVMCreateStringAttribute(state_.context_, key, key_len, value,
                                   value_len);
}

int Output::RegisterToParameterLoc(Register r) {
#if defined(TARGET_ARCH_ARM)
  // all register listed in the parameter
  return r;
#elif defined(TARGET_ARCH_ARM64)
  int r_value = r;
  // [X0, X7] listed in the arguments
  if (r_value < 8) return r_value;
  switch (r) {
    case DISPATCH_TABLE_REG:
      return 8;
    case NULL_REG:
      return 9;
    case CODE_REG:
      return 10;
    case kWriteBarrierSlotReg:
      return 11;
    case THR:
      return 12;
    case PP:
      return 13;
    case BARRIER_MASK:
      return 14;
    default:
      UNREACHABLE();
  }
#endif
}
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
