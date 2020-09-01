// Copyright 2019 UCWeb Co., Ltd.

#ifndef OUTPUT_H
#define OUTPUT_H
#include <string>
#include <unordered_map>
#include <vector>

#include "vm/compiler/backend/llvm/intrinsic_repository.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include "vm/compiler/backend/llvm/stack_map_info.h"

namespace dart {
class Instruction;
namespace dart_llvm {
struct CompilerState;
struct RegisterParameter {
  int name;
  LType type;
  RegisterParameter(int _name, LType _type) : name(_name), type(_type) {}
};

using RegisterParameterDesc = std::vector<RegisterParameter>;

class Output {
 public:
  Output(CompilerState& state);
  ~Output();
  void initializeBuild(const RegisterParameterDesc&, LType return_type);
  LBasicBlock appendBasicBlock(const char* name = "");
  LBasicBlock appendBasicBlock(LValue function, const char* name = "");
  LBasicBlock getInsertionBlock();
  void positionToBBEnd(LBasicBlock);
  void positionBefore(LValue);
  LValue constInt8(int);
  LValue constInt16(int);
  LValue constInt32(int);
  LValue constIntPtr(intptr_t);
  LValue constInt64(long long);
  LValue constDouble(double);
  LValue constTagged(uintptr_t);
  LValue constString(const char*);
  LValue buildStructGEP(LValue structVal, unsigned field);
  LValue buildGEPWithByteOffset(LValue base, LValue offset, LType dstType);
  LValue buildGEPWithByteOffset(LValue base, int offset, LType dstType);
  LValue buildGEP(LValue base, LValue offset);
  LValue buildLoad(LValue toLoad);
  LValue buildInvariantLoad(LValue toLoad);
  LValue buildLoadUnaligned(LValue toLoad);
  LValue buildLoadSmi(LValue toLoad);
  LValue buildStore(LValue val, LValue pointer);
  LValue buildNeg(LValue val);
  LValue buildAdd(LValue lhs, LValue rhs);
  LValue buildFAdd(LValue lhs, LValue rhs);
  LValue buildNSWAdd(LValue lhs, LValue rhs);
  LValue buildSub(LValue lhs, LValue rhs);
  LValue buildFSub(LValue lhs, LValue rhs);
  LValue buildNSWSub(LValue lhs, LValue rhs);
  LValue buildMul(LValue lhs, LValue rhs);
  LValue buildSRem(LValue lhs, LValue rhs);
  LValue buildSDiv(LValue lhs, LValue rhs);
  LValue buildFMul(LValue lhs, LValue rhs);
  LValue buildFDiv(LValue lhs, LValue rhs);
  LValue buildFCmp(LRealPredicate cond, LValue lhs, LValue rhs);
  LValue buildFNeg(LValue input);
  LValue buildNSWMul(LValue lhs, LValue rhs);
  LValue buildShl(LValue lhs, LValue rhs);
  LValue buildShr(LValue lhs, LValue rhs);
  LValue buildSar(LValue lhs, LValue rhs);
  LValue buildAnd(LValue lhs, LValue rhs);
  LValue buildOr(LValue lhs, LValue rhs);
  LValue buildXor(LValue lhs, LValue rhs);
  LValue buildNot(LValue v);
  LValue buildBr(LBasicBlock bb);
  LValue buildSwitch(LValue, LBasicBlock, unsigned);
  LValue buildCondBr(LValue condition, LBasicBlock taken, LBasicBlock notTaken);
  LValue buildRet(LValue ret);
  void buildRetUndef();
  void buildReturnForTailCall();
  LValue buildRetVoid(void);
  LValue buildSelect(LValue condition, LValue taken, LValue notTaken);
  LValue buildICmp(LIntPredicate cond, LValue left, LValue right);
  LValue buildPhi(LType type);
  LValue buildAlloca(LType);

  LValue buildCall(LValue function, const LValue* args, unsigned numArgs);

  template <typename VectorType>
  inline LValue buildCall(LValue function, const VectorType& vector) {
    return buildCall(function, vector.begin(), vector.size());
  }
  inline LValue buildCall(LValue function) {
    return buildCall(function, nullptr, 0U);
  }
  inline LValue buildCall(LValue function, LValue arg1) {
    return buildCall(function, &arg1, 1);
  }
  template <typename... Args>
  LValue buildCall(LValue function, LValue arg1, Args... args) {
    LValue argsArray[] = {arg1, args...};
    return buildCall(function, argsArray, sizeof(argsArray) / sizeof(LValue));
  }

  LValue buildInvoke(LValue function,
                     const LValue* args,
                     unsigned numArgs,
                     LBasicBlock then,
                     LBasicBlock exception);

  LValue buildCast(LLVMOpcode Op, LLVMValueRef Val, LLVMTypeRef DestTy);
  LValue buildBitCast(LValue val, LType type);
  LValue buildPointerCast(LValue val, LType type);
  LValue getStatePointFunction(LType callee_type);
  LValue getGCResultFunction(LType return_type);

  LValue buildInlineAsm(LType, char*, size_t, char*, size_t, bool);

  void buildUnreachable();
  LValue buildExtractValue(LValue aggVal, unsigned index);
  LValue buildInsertValue(LValue aggVal, unsigned index, LValue value);
  LValue buildLandingPad();
  LLVMAttributeRef createStringAttr(const char* key,
                                    unsigned key_len,
                                    const char* value,
                                    unsigned value_len);
  void assignEvenNumberAttr(LValue load);
  void assignCSRAttr(LValue call, const char* csr);
  void setDebugInfo(intptr_t linenum, const char* source_file_name);
  void finalize();
  LValue addFunction(const char* name, LType type);
  LType tagged_pair() const;
  void EmitDebugInfo(std::vector<Instruction*>&& debug_instrs);
  void EmitStackMapInfoMap(StackMapInfoMap&& stack_map_info_map);
  LValue GetRegisterParameter(Register r);

  inline IntrinsicRepository& repo() { return repo_; }
  inline LBasicBlock prologue() const { return prologue_; }
  inline LType tagged_type() const { return repo_.tagged_type; }
  inline LValue parameter(int i) { return parameters_[i]; }
  inline LValue thread() { return thread_; }
  // FIXME: this is not allow. it will escape the gc relocator.
  inline LValue args_desc() { return args_desc_; }
  LValue fp();
  inline LValue pp() { return pp_; }
  inline LValue bitcast_space() { return bitcast_space_; }
  inline int stack_parameter_count() const { return stack_parameter_count_; }
  static int RegisterToParameterLoc(Register);

 private:
  LValue setInstrDebugLoc(LValue);
  void AddFunctionCommonAttr(LValue function);
  void finalizeDebugInfo();
  void initializeFunction(const RegisterParameterDesc&, LType return_type);
  CompilerState& state_;
  IntrinsicRepository repo_;
  LBuilder builder_;
  LLVMDIBuilderRef di_builder_;
  LBasicBlock prologue_;
  LValue thread_;
  LValue args_desc_;
  LValue pp_;
  LValue bitcast_space_;
  LType return_type_;
  LLVMMetadataRef subprogram_;
  size_t stack_parameter_count_;

  std::vector<LValue> parameters_;
  std::unordered_map<LType, LValue> gc_function_map_;
};
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
#endif  /* OUTPUT_H */
