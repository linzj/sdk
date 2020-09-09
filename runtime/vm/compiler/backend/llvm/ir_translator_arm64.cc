bool AnonImpl::support_integer_div() const {
  return true;
}

void AnonImpl::AdjustInstrSizeForLoadFromConsecutiveAddress(
    size_t& instr_size,
    intptr_t offset) const {
  static const constexpr size_t increment = Instr::kInstrSize;
  const uint32_t upper20 = offset & 0xfffff000;
  const uint32_t lower12 = offset & 0x00000fff;
  compiler::Operand op;
  if (LIKELY(compiler::Address::CanHoldOffset(offset,
                                              compiler::Address::PairOffset))) {
    return;
  }
  if (compiler::Operand::CanHold(offset, kXRegSizeInBits, &op) ==
      compiler::Operand::Immediate) {
    instr_size += increment;
  } else if (compiler::Operand::CanHold(upper20, kXRegSizeInBits, &op) ==
                 compiler::Operand::Immediate &&
             compiler::Address::CanHoldOffset(lower12,
                                              compiler::Address::PairOffset)) {
    instr_size += increment;
  } else {
    instr_size += increment;
    instr_size += increment;
  }
}

void AnonImpl::AdjustInstrSizeForLoad(size_t& instr_size,
                                      intptr_t offset) const {
  static const constexpr size_t increment = Instr::kInstrSize;
  const uint32_t upper20 = offset & 0xfffff000;
  compiler::Operand op;
  if (LIKELY(compiler::Address::CanHoldOffset(offset))) {
    return;
  } else if (compiler::Operand::CanHold(upper20, kXRegSizeInBits, &op) ==
             compiler::Operand::Immediate) {
    instr_size += increment;
  } else {
    instr_size += increment;
    instr_size += increment;
  }
}

LValue AnonImpl::LoadObjectFromPool(intptr_t offset) {
  LValue gep = output().buildGEPWithByteOffset(
      GetPPValue(), output().constIntPtr(offset),
      pointerType(output().tagged_type()));
  return output().buildLoad(gep);
}

const char* AnonImpl::GetSharedStubCSR() {
  return "x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,"
         "x15,x18,x19,x20,x21,x22,x24,x26,x27,x28,q0,q1,"
         "q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,"
         "q17,q18,q19,q20,q21,q22,q23,q24,q25,q26,q27,q28,q29,"
         "q30,q31";
}

const char* AnonImpl::GetCCallCSR() {
  return "x19,x20,x21,x22,x24,x26,x27,x28,q8,q9,q10,q11,q12,"
         "q13,q14,q15";
}

const char* AnonImpl::GetBoxInt64SharedStubCSR() {
  return "x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x18,"
         "x19,x20,x21,x22,x24,x25,x26,x27,x28,q0,q1,q2,q3,"
         "q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,q17,q18,"
         "q19,q20,q21,q22,q23,q24,q25,q26,q27,q28,q29,q30,q31";
}
