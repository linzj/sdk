bool AnonImpl::support_integer_div() const {
  return TargetCPUFeatures::integer_division_supported();
}

static void AdjustAddress(size_t& instr_size, intptr_t offset) {
  static const constexpr size_t increment = Instr::kInstrSize;
  int32_t offset_mask = 0;
  offset -= kHeapObjectTag;
  if (LIKELY(
          compiler::Address::CanHoldLoadOffset(kWord, offset, &offset_mask))) {
    return;
  } else {
    int32_t offset_hi = offset & ~offset_mask;  // signed
    compiler::Operand o;
    if (compiler::Operand::CanHold(offset_hi, &o)) {
      instr_size += increment;
    } else {
      instr_size += 2 * increment;
    }
  }
}

void AnonImpl::AdjustInstrSizeForLoadFromConsecutiveAddress(
    size_t& instr_size,
    intptr_t offset) const {
  AdjustAddress(instr_size, offset);
  AdjustAddress(instr_size, offset + compiler::target::kWordSize);
}

void AnonImpl::AdjustInstrSizeForLoad(size_t& instr_size,
                                      intptr_t offset) const {
  AdjustAddress(instr_size, offset);
}

LValue AnonImpl::LoadObjectFromPool(intptr_t offset) {
  LValue gep = output().buildGEPWithByteOffset(
      GetPPValue(), output().constIntPtr(offset - kHeapObjectTag),
      pointerType(output().tagged_type()));
  return output().buildLoad(gep);
}

const char* AnonImpl::GetSharedStubCSR() {
  return "r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,d0,d1,d2,d3,"
         "d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18,"
         "d19,d20,d21,d22,d23,d24,d25,d26,d27,d28,d29,d30,d31";
}

const char* AnonImpl::GetCCallCSR() {
  return nullptr;
}

const char* AnonImpl::GetBoxInt64SharedStubCSR() {
  return "r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,d0,d1,d2,d3,d4,d5,"
         "d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18,d19,"
         "d20,d21,d22,d23,d24,d25,d26,d27,d28,d29,d30,d31";
}
