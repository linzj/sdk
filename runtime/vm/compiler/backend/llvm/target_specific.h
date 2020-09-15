// Copyright 2019 UCWeb Co., Ltd.
#ifndef TARGET_SPECIFIC_H
#define TARGET_SPECIFIC_H
#include "vm/compiler/backend/llvm/llvm_config.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include "vm/constants.h"
namespace dart {
#if defined(TARGET_ARCH_ARM)
static const int kV8CCRegisterParameterCount = 12;
static const int kDwarfGenernalRegEnd = 15;

static const Register kReceiverReg = R0;
static const Register kICReg = R9;

// Reg Call
static const Register kCallTargetReg = R12;
static const size_t kCallInstrSize = Instr::kInstrSize;
static const size_t kCallReturnOnStackInstrSize = 2 * Instr::kInstrSize;

// Runtime Call
static const Register kRuntimeCallArgCountReg = R4;
static const Register kRuntimeCallEntryReg = R9;

// NativeCall
static const Register kNativeArgcReg = R1;
static const size_t kNativeCallInstrSize = 6 * Instr::kInstrSize;
static const size_t kLoadInstrSize = Instr::kInstrSize;

// Thread offset Call
static const size_t kThreadOffsetCallInstrSize =
    kCallInstrSize + Instr::kInstrSize;

// Instance Call
static const size_t kInstanceCallInstrSize = 3 * Instr::kInstrSize;

// InvokeCFunction
static const size_t kInvokeCFunctionCallInstrSize = 5 * Instr::kInstrSize;

// instance of
static const Register kInstanceOfInstanceReg = R0;
static const Register kInstanceOfFunctionTypeReg = R1;
static const Register kInstanceOfInstantiatorTypeReg = R2;

// array
static const Register kCreateArrayLengthReg = R2;
static const Register kCreateArrayElementTypeReg = R1;

// Context Allocation
static const Register kAllocateContextNumOfContextVarsReg = R1;

static const int kDwarfSP = SP;
static const int kDwarfFP = FP;
static const int kDwarfFPFixup = 1;
static const int kDwarfSPForSafePoint = SP;
#define TARGET_SUPPORT_DISPATCH_TABLE_REG
#elif defined(TARGET_ARCH_ARM64)
static const int kV8CCRegisterParameterCount = 15;
static const int kDwarfGenernalRegEnd = 30;

static const Register kReceiverReg = R0;
static const Register kICReg = R5;

// Reg Call
static const Register kCallTargetReg = LR;
static const size_t kCallInstrSize = Instr::kInstrSize;
static const size_t kCallReturnOnStackInstrSize = 2 * Instr::kInstrSize;

// Runtime Call
static const Register kRuntimeCallArgCountReg = R4;
static const Register kRuntimeCallEntryReg = R5;

// NativeCall
static const Register kNativeArgcReg = R1;
static const size_t kNativeCallInstrSize = 6 * Instr::kInstrSize;
static const size_t kLoadInstrSize = Instr::kInstrSize;

// Thread offset Call
static const size_t kThreadOffsetCallInstrSize =
    kCallInstrSize + Instr::kInstrSize;

// Instance Call
static const size_t kInstanceCallInstrSize = 2 * Instr::kInstrSize;

// InvokeCFunction
static const size_t kInvokeCFunctionCallInstrSize = 11 * Instr::kInstrSize;

// instance of
static const Register kInstanceOfInstanceReg = R0;
static const Register kInstanceOfFunctionTypeReg = R1;
static const Register kInstanceOfInstantiatorTypeReg = R2;

// array
static const Register kCreateArrayLengthReg = R2;
static const Register kCreateArrayElementTypeReg = R1;

// Context Allocation
static const Register kAllocateContextNumOfContextVarsReg = R1;

static const int kDwarfSP = 15;
static const int kDwarfFP = 29;
static const int kDwarfFPFixup = 1;
static const int kDwarfSPForSafePoint = 31;
#define TARGET_SUPPORT_DISPATCH_TABLE_REG
#define TARGET_SUPPORT_NULL_OBJECT_REG
#define TARGET_SUPPORT_BARRIER_MASK_REG
#else
#error unsupported arch
#endif
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
#endif  // TARGET_SPECIFIC_H
