// Copyright 2019 UCWeb Co., Ltd.

#ifndef STACK_MAP_INFO_H
#define STACK_MAP_INFO_H
#include <stdint.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "vm/compiler/backend/llvm/llvm_config.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include "vm/compiler/backend/llvm/llvm_log.h"
#include "vm/compiler/backend/locations.h"
#include "vm/object.h"
#include "vm/token_position.h"

namespace dart {
namespace dart_llvm {

enum class StackMapInfoType {
  kCallInfo,
};

class StackMapInfo {
 public:
  explicit StackMapInfo(StackMapInfoType);
  virtual ~StackMapInfo();

  StackMapInfoType GetType() const { return type_; }
  int patchid() const { return patchid_; }
  void set_patchid(int _patchid) { patchid_ = _patchid; }

 private:
  const StackMapInfoType type_;
  int patchid_;
};

#define DEFINE_ACCESSOR(type, name, class_name)                                \
  type name() const { return name##_; }                                        \
  class_name* set_##name(type _##name) {                                       \
    name##_ = _##name;                                                         \
    return this;                                                               \
  }

class CallSiteInfo final : public StackMapInfo {
 public:
  enum class CallTargetType {
    kUnspecify,
    // target passed in the GP reg.
    kReg,
    // relative call target in the function
    kCallRelative,
    // relative call target in the code
    kStubRelative,
    kThreadOffset,
    kNative,
    kPatchable,
    kCCall,
  };
  explicit CallSiteInfo();
  ~CallSiteInfo() override = default;
#define CALLSITE_ACCESSOR(V)                                                   \
  V(CallTargetType, type)                                                      \
  V(TokenPosition, token_pos)                                                  \
  V(intptr_t, deopt_id)                                                        \
  V(size_t, stack_parameter_count)                                             \
  V(size_t, instr_size)                                                        \
  V(intptr_t, try_index)                                                       \
  V(RawPcDescriptors::Kind, kind)                                              \
  V(const Function*, target)                                                   \
  V(const Code*, code)                                                         \
  V(const Code*, fpu_code)                                                     \
  V(CodeEntryKind, entry_kind)                                                 \
  V(bool, is_tailcall)                                                         \
  V(bool, needs_metadata)                                                      \
  V(int, return_on_stack_pos)                                                  \
  V(int64_t, parameter_bits)                                                   \
  V(int, valid_bits)                                                           \
  V(intptr_t, thread_offset)                                                   \
  V(intptr_t, fpu_thread_offset)                                               \
  V(intptr_t, native_entry_pool_offset)                                        \
  V(intptr_t, stub_pool_offset)                                                \
  V(intptr_t, ic_pool_offset)                                                  \
  V(intptr_t, target_stub_pool_offset)                                         \
  V(const RuntimeEntry*, runtime_entry)

#define CALLSITE_WAPPER(type, name) DEFINE_ACCESSOR(type, name, CallSiteInfo)

  CALLSITE_ACCESSOR(CALLSITE_WAPPER)

#undef CALLSITE_WAPPER
#undef CALLSITE_ACCESSOR
  void MarkParameterBit(int which, bool set);

 private:
  CallTargetType type_;
  TokenPosition token_pos_;
  intptr_t deopt_id_;
  // for stack maps generate, we will mark the lowest stack_parameter_count's stack slot 1.
  size_t stack_parameter_count_;
  size_t instr_size_;
  intptr_t try_index_;
  RawPcDescriptors::Kind kind_;
  union {
    struct {
      const Function* target_;
      Code::EntryKind entry_kind_;
    };
    // shared stubs
    struct {
      const Code* code_;
      const Code* fpu_code_;
    };
    // thread offset shared stubs
    struct {
      intptr_t thread_offset_;
      intptr_t fpu_thread_offset_;
    };
    // native calls
    struct {
      intptr_t native_entry_pool_offset_;
      intptr_t stub_pool_offset_;
    };
    // patchable calls
    struct {
      intptr_t ic_pool_offset_;
      intptr_t target_stub_pool_offset_;
    };
    // C call
    struct {
      const RuntimeEntry* runtime_entry_;
    };
  };
  int64_t parameter_bits_;
  int valid_bits_;
  int return_on_stack_pos_;
  bool is_tailcall_;
  bool needs_metadata_;
};

#undef DEFINE_ACCESSOR
typedef std::unordered_map<int, std::unique_ptr<StackMapInfo>> StackMapInfoMap;
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
#endif  // STACK_MAP_INFO_H
