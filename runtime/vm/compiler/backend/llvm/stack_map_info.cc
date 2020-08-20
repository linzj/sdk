// Copyright 2019 UCWeb Co., Ltd.

#include "vm/compiler/backend/llvm/stack_map_info.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include "vm/code_descriptors.h"

namespace dart {
namespace dart_llvm {
StackMapInfo::StackMapInfo(StackMapInfoType type) : type_(type), patchid_(0) {}
StackMapInfo::~StackMapInfo() {}

CallSiteInfo::CallSiteInfo()
    : StackMapInfo(StackMapInfoType::kCallInfo),
      type_(CallTargetType::kUnspecify),
      token_pos_(),
      deopt_id_(0),
      stack_parameter_count_(0),
      instr_size_(0),
      try_index_(kInvalidTryIndex),
      kind_(RawPcDescriptors::kOther),
      code_(nullptr),
      parameter_bits_(0),
      valid_bits_(0),
      return_on_stack_pos_(-1),
      is_tailcall_(false),
      needs_metadata_(true) {}

void CallSiteInfo::MarkParameterBit(int which, bool set) {
  EMASSERT(which < 64);
  if (set) parameter_bits_ |= 1ULL << which;
  if (which + 1 > valid_bits_) valid_bits_ = which + 1;
}
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
