#include "vm/compiler/backend/llvm/initialize_llvm.h"
#if defined(DART_ENABLE_LLVM_COMPILER)
#include <assert.h>
#include <llvm-c/Support.h>
#include <stdio.h>

#include "vm/compiler/backend/llvm/llvm_headers.h"
#include "vm/compiler/backend/llvm/llvm_log.h"

namespace dart {
namespace dart_llvm {
static void LLVMCrash(const char*) __attribute__((noreturn));

void LLVMCrash(const char* reason) {
  fprintf(stderr, "LLVM fatal error: %s", reason);
  EMASSERT(false);
  __builtin_unreachable();
}

static void InitializeAndGetLLVMAPI(void) {
  LLVMInstallFatalErrorHandler(LLVMCrash);
  const char* options[] = {
    "v8 builtins compiler",
#if defined(FEATURE_USE_SAMPLE_PGO)
    "-sample-profile-file=../../v8/src/llvm/sample.prof",
#endif
    "-vectorize-loops",
    "-runtime-memory-check-threshold=16",
  };
  LLVMParseCommandLineOptions(sizeof(options) / sizeof(const char*), options,
                              nullptr);

  // You think you want to call LLVMInitializeNativeTarget()? Think again. This
  // presumes that LLVM was ./configured correctly, which won't be the case in
  // cross-compilation situations.
  LLVMLinkInMCJIT();
#if defined(TARGET_ARCH_ARM)
#define TARGET_NAME ARM
#elif defined(TARGET_ARCH_ARM64)
#define TARGET_NAME AArch64
#else
#error unsupported arch
#endif

#define INITIALIZE_TARGET(target_name)                                         \
  LLVMInitialize##target_name##TargetInfo();                                   \
  LLVMInitialize##target_name##Target();                                       \
  LLVMInitialize##target_name##TargetMC();                                     \
  LLVMInitialize##target_name##AsmPrinter();                                   \
  LLVMInitialize##target_name##Disassembler();                                 \
  LLVMInitialize##target_name##AsmParser();

#define CALL_INITIALIZE(func, name) func(name)

  CALL_INITIALIZE(INITIALIZE_TARGET, TARGET_NAME)

#undef CALL_INITIALIZE
#undef INITIALIZE_TARGET
#undef TARGET_NAME
}

namespace {
class LLVMInitializer {
 public:
  LLVMInitializer();
  ~LLVMInitializer() = default;
};

LLVMInitializer::LLVMInitializer() {
  InitializeAndGetLLVMAPI();
}
}  // namespace

void InitLLVM(void) {
  static LLVMInitializer initializer;
}
}  // namespace dart_llvm
}  // namespace dart
#endif  // DART_ENABLE_LLVM_COMPILER
