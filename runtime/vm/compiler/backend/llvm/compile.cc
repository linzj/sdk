// Copyright 2019 UCWeb Co., Ltd.

#include "vm/compiler/backend/llvm/compile.h"

#include "vm/compiler/backend/llvm/compiler_state.h"
#include "vm/compiler/backend/llvm/llvm_log.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include <assert.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <memory>

#include "vm/compiler/backend/il.h"
#include "vm/compiler/backend/llvm/dwarf_info.h"

#define SECTION_NAME_PREFIX "."
#define SECTION_NAME(NAME) (SECTION_NAME_PREFIX NAME)

namespace dart {
namespace dart_llvm {
typedef CompilerState State;

static const char* symbolLookupCallback(void* DisInfo,
                                        uint64_t ReferenceValue,
                                        uint64_t* ReferenceType,
                                        uint64_t ReferencePC,
                                        const char** ReferenceName) {
  *ReferenceType = LLVMDisassembler_ReferenceType_InOut_None;
  return nullptr;
}

static void disassemble(CompilerState& state,
                        ByteBuffer& code,
                        const DwarfLineMapper& mapper) {
#if defined(TARGET_ARCH_ARM)
  LLVMDisasmContextRef DCR = LLVMCreateDisasm(
      "armv8-unknown-unknown-v8", nullptr, 0, nullptr, symbolLookupCallback);
#elif defined(TARGET_ARCH_ARM64)
  LLVMDisasmContextRef DCR = LLVMCreateDisasm(
      "aarch64-unknown-unknown-v8", nullptr, 0, nullptr, symbolLookupCallback);
#else
#error unsupport arch
#endif

  uint8_t* BytesP = code.data();

  unsigned NumBytes = code.size();
  unsigned PC = 0;
  const char OutStringSize = 100;
  char OutString[OutStringSize];
  printf(
      "========================================================================"
      "========\n");
  auto& debug_map = mapper.GetMap();
  auto& debug_instrs_ = state.debug_instrs_;
  while (NumBytes != 0) {
    auto found = debug_map.find(PC);
    if (found != debug_map.end() && found->second > 1) {
      int index = found->second - 2;
      printf("%s\n", debug_instrs_[index]->ToCString());
    }
    size_t InstSize = LLVMDisasmInstruction(DCR, BytesP, NumBytes, PC,
                                            OutString, OutStringSize);
    if (InstSize == 0) {
      printf("%08x: %08x maybe constant\n", PC,
             *reinterpret_cast<uint32_t*>(BytesP));
      PC += 4;
      BytesP += 4;
      NumBytes -= 4;
    }
    printf("%08x: %08x %s\n", PC, *reinterpret_cast<uint32_t*>(BytesP),
           OutString);
    PC += InstSize;
    BytesP += InstSize;
    NumBytes -= InstSize;
  }
  printf(
      "========================================================================"
      "========\n");
  LLVMDisasmDispose(DCR);
}

static DART_UNUSED void disassemble(CompilerState& state,
                                    const DwarfLineMapper& mapper) {
  printf("Disassemble for function:%s\n", state.function_name_.c_str());
  for (auto& code : state.code_section_list_) {
    disassemble(state, *code, mapper);
  }
}

static uint8_t* mmAllocateCodeSection(void* opaqueState,
                                      uintptr_t size,
                                      unsigned alignment,
                                      unsigned sid,
                                      const char* sectionName) {
  State& state = *static_cast<State*>(opaqueState);
  dart_llvm::ByteBuffer* bb = state.AllocateByteBuffer(size);
  state.code_section_list_.push_back(bb);

  state.AddSection(sid, sectionName, bb, alignment, true);

  return const_cast<uint8_t*>(bb->data());
}

static uint8_t* mmAllocateDataSection(void* opaqueState,
                                      uintptr_t size,
                                      unsigned alignment,
                                      unsigned sid,
                                      const char* sectionName,
                                      LLVMBool) {
  State& state = *static_cast<State*>(opaqueState);

  dart_llvm::ByteBuffer* bb = state.AllocateByteBuffer(size);
  state.data_section_list_.push_back(bb);

#if defined(TARGET_ARCH_ARM)
  static const char kExceptionTablePrefix[] = SECTION_NAME("ARM.extab");
#elif defined(TARGET_ARCH_ARM64)
  static const char kExceptionTablePrefix[] = SECTION_NAME("gcc_except_table");
#else
#error unsupport arch
#endif
  static const char kDwarfLine[] = ".debug_line";
  if (!strcmp(sectionName, SECTION_NAME("llvm_stackmaps"))) {
    state.stackMapsSection_ = bb;
  } else if (!memcmp(sectionName, kExceptionTablePrefix,
                     sizeof(kExceptionTablePrefix) - 1)) {
    state.exception_table_ = bb;
  } else if (!memcmp(sectionName, kDwarfLine, sizeof(kDwarfLine) - 1)) {
    state.dwarf_line_ = bb;
  }

  state.AddSection(sid, sectionName, bb, alignment, false);
  return const_cast<uint8_t*>(bb->data());
}

static LLVMBool mmApplyPermissions(void*, char**) {
  return false;
}

static void mmDestroy(void*) {}
#if defined(FEATURE_SAVE_OBJECT_FILE)
// This must be kept in sync with gdb/gdb/jit.h .
typedef enum {
  JIT_NOACTION = 0,
  JIT_REGISTER_FN,
  JIT_UNREGISTER_FN
} jit_actions_t;

struct jit_code_entry {
  struct jit_code_entry* next_entry;
  struct jit_code_entry* prev_entry;
  const char* symfile_addr;
  uint64_t symfile_size;
};

struct jit_descriptor {
  uint32_t version;
  // This should be jit_actions_t, but we want to be specific about the
  // bit-width.
  uint32_t action_flag;
  struct jit_code_entry* relevant_entry;
  struct jit_code_entry* first_entry;
};

extern "C" struct jit_descriptor __jit_debug_descriptor;
void SaveObjectFile(const State& state) {
  if (__jit_debug_descriptor.action_flag != JIT_REGISTER_FN) {
    return;
  }
  char file_name_buf[256];
  snprintf(file_name_buf, 256, "%s.o", state.function_name_.c_str());
  int fd = open(file_name_buf, O_WRONLY | O_CREAT | O_EXCL,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
  if (fd == -1) return;
  write(fd, __jit_debug_descriptor.first_entry->symfile_addr,
        __jit_debug_descriptor.first_entry->symfile_size);
  close(fd);
}
#endif  // defined(FEATURE_SAVE_OBJECT_FILE)

void Compile(State& state) {
  LLVMMCJITCompilerOptions options;
  LLVMInitializeMCJITCompilerOptions(&options, sizeof(options));
  options.OptLevel = 3;
#if defined(TARGET_ARCH_ARM64)
  options.CodeModel = LLVMCodeModelTiny;
#endif
  LLVMExecutionEngineRef engine;
  char* error = 0;
  options.MCJMM = LLVMCreateSimpleMCJITMemoryManager(
      &state, mmAllocateCodeSection, mmAllocateDataSection, mmApplyPermissions,
      mmDestroy);
  if (LLVMCreateMCJITCompilerForModule(&engine, state.module_, &options,
                                       sizeof(options), &error)) {
    LLVMLOGE("FATAL: Could not create LLVM execution engine: %s", error);
    EMASSERT(false);
  }
  LLVMModuleRef module = state.module_;
  LLVMPassManagerRef functionPasses = 0;
  LLVMPassManagerRef modulePasses;
  LLVMTargetDataRef targetData = LLVMGetExecutionEngineTargetData(engine);
  char* stringRepOfTargetData = LLVMCopyStringRepOfTargetData(targetData);
  LLVMSetDataLayout(module, stringRepOfTargetData);
  free(stringRepOfTargetData);

  LLVMPassManagerBuilderRef passBuilder = LLVMPassManagerBuilderCreate();
  LLVMPassManagerBuilderSetOptLevel(passBuilder, 3);
  LLVMPassManagerBuilderSetSizeLevel(passBuilder, 1);

  functionPasses = LLVMCreateFunctionPassManagerForModule(module);
  modulePasses = LLVMCreatePassManager();

  LLVMTargetLibraryInfoRef TLI = LLVMCreateEmptyTargetLibraryInfo();
  LLVMAddTargetLibraryInfo(TLI, functionPasses);
  LLVMAddTargetLibraryInfo(TLI, modulePasses);
  LLVMTargetMachineRef target_machine =
      LLVMGetExecutionEngineTargetMachine(engine);
  LLVMAddAnalysisPasses(target_machine, functionPasses);
  LLVMAddAnalysisPasses(target_machine, modulePasses);
  LLVMPassManagerBuilderPopulateFunctionPassManager(passBuilder,
                                                    functionPasses);
  LLVMPassManagerBuilderPopulateModulePassManager(passBuilder, modulePasses);

  LLVMPassManagerBuilderDispose(passBuilder);

  LLVMInitializeFunctionPassManager(functionPasses);
  for (LLVMValueRef function = LLVMGetFirstFunction(module); function;
       function = LLVMGetNextFunction(function))
    LLVMRunFunctionPassManager(functionPasses, function);
  LLVMFinalizeFunctionPassManager(functionPasses);

  LLVMRunPassManager(modulePasses, module);
  state.entryPoint_ =
      reinterpret_cast<void*>(LLVMGetPointerToGlobal(engine, state.function_));

  if (functionPasses) LLVMDisposePassManager(functionPasses);
  LLVMDisposePassManager(modulePasses);
#if defined(FEATURE_SAVE_OBJECT_FILE)
  SaveObjectFile(state);
#endif  // defined(FEATURE_SAVE_OBJECT_FILE)
  LLVMDisposeExecutionEngine(engine);
#if 0
  DwarfLineMapper mapper;
  mapper.Process(state.dwarf_line_->data());
  disassemble(state, mapper);
#endif
}
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
