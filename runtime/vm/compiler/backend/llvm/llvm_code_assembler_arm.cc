void CodeAssembler::CallWithCallReg(const CallSiteInfo* call_site_info,
                                    dart::Register reg) {
  if (LIKELY(!call_site_info->is_tailcall()))
    assembler().blx(reg);
  else
    assembler().bx(reg);
}

void CodeAssembler::GenerateNativeCall(const CallSiteInfo* call_site_info) {
  assembler().add(R2, SP,
                  compiler::Operand(call_site_info->stack_parameter_count() *
                                    compiler::target::kWordSize));
  assembler().LoadWordFromPoolOffset(
      R9, call_site_info->native_entry_pool_offset() - kHeapObjectTag, PP, AL);
  assembler().LoadWordFromPoolOffset(
      CODE_REG, call_site_info->stub_pool_offset() - kHeapObjectTag, PP, AL);
  assembler().ldr(LR, compiler::FieldAddress(
                          CODE_REG, compiler::target::Code::entry_point_offset(
                                        CodeEntryKind::kNormal)));
  assembler().blx(
      LR);  // Use blx instruction so that the return branch prediction works.
}

void CodeAssembler::GeneratePatchableCall(const CallSiteInfo* call_site_info) {
  assembler().LoadWordFromPoolOffset(
      LR, call_site_info->target_stub_pool_offset() - kHeapObjectTag, PP, AL);
  assembler().LoadWordFromPoolOffset(
      R9, call_site_info->ic_pool_offset() - kHeapObjectTag, PP, AL);
  assembler().blx(
      LR);  // Use blx instruction so that the return branch prediction works.
}

void CodeAssembler::PrepareLoadCPAction() {}
void CodeAssembler::EmitCP() {}

struct CodeAssembler::ArchImpl {};

static int BranchOffset(const void* code) {
  uint32_t insn = *reinterpret_cast<const uint32_t*>(code);
  unsigned op1 = (insn >> 24) & 0xf;
  switch (op1) {
    case 0xa:
    case 0xb: {
      int32_t offset;

      /* branch (and link) */
      if (insn & (1 << 24)) {
        break;
      }
      offset = Sextract32(insn << 2, 0, 26);
      offset += 8;
      return offset;
    }
    default:
      break;
  }
  return -1;
}
