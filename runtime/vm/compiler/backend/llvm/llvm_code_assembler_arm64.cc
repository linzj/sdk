void CodeAssembler::CallWithCallReg(const CallSiteInfo* call_site_info,
                                    dart::Register reg) {
  if (LIKELY(!call_site_info->is_tailcall()))
    assembler().blr(reg);
  else
    assembler().br(reg);
}

void CodeAssembler::GenerateNativeCall(const CallSiteInfo* call_site_info) {
  EMASSERT(call_site_info->native_entry_pool_offset() +
               compiler::target::kWordSize ==
           call_site_info->stub_pool_offset());
  assembler().add(R2, SP,
                  compiler::Operand(call_site_info->stack_parameter_count() *
                                    compiler::target::kWordSize));
  assembler().LoadWordFromPoolOffset(
      R5, call_site_info->native_entry_pool_offset());
  assembler().LoadWordFromPoolOffset(CODE_REG,
                                     call_site_info->stub_pool_offset());
  assembler().ldr(TMP, compiler::FieldAddress(
                           CODE_REG, compiler::target::Code::entry_point_offset(
                                         CodeEntryKind::kNormal)));
  assembler().blr(
      TMP);  // Use blx instruction so that the return branch prediction works.
}

void CodeAssembler::GeneratePatchableCall(const CallSiteInfo* call_site_info) {
  EMASSERT(call_site_info->ic_pool_offset() + compiler::target::kWordSize ==
           call_site_info->target_stub_pool_offset());
  assembler().LoadDoubleWordFromPoolOffset(R5, LR,
                                           call_site_info->ic_pool_offset());
  assembler().blr(
      LR);  // Use blx instruction so that the return branch prediction works.
}

static size_t RoundUp(size_t v, size_t align_shift) {
  v += (1U << align_shift) - 1;
  v >>= align_shift;
  v <<= align_shift;
  return v;
}

namespace {
class ConstantPoolResolver {
 public:
  ConstantPoolResolver() = default;
  ~ConstantPoolResolver() = default;

  bool Prepare(const CompilerState& state, size_t code_size);
  void EmitCP(compiler::Assembler& assembler);
  size_t ResolveOffset(unsigned offset,
                       unsigned sym_index,
                       unsigned addend,
                       bool is_64);

 private:
  const CompilerState& compiler_state() const { return *state_; }
  unsigned GetSectionId64(unsigned sym_index);

  const CompilerState* state_;
  std::vector<const Section*> ro_sections_;
  size_t aligned_code_size_;
  const Section* symbol_section_ = nullptr;
  DISALLOW_COPY_AND_ASSIGN(ConstantPoolResolver);
};

bool ConstantPoolResolver::Prepare(const CompilerState& state,
                                   size_t code_size) {
  // find all the ro data
  state_ = &state;
  size_t aligned_shift = 2;
  for (const auto& s : compiler_state().sections_) {
    if (s.name.find(".rodata") == 0) {
      ro_sections_.emplace_back(&s);
      size_t section_aligned_shift = log2(s.alignment);
      if (section_aligned_shift > aligned_shift)
        aligned_shift = section_aligned_shift;
    } else if (s.name == ".symtab")
      symbol_section_ = &s;
  }
  if (ro_sections_.empty()) return false;
  EMASSERT(symbol_section_ != nullptr);
  aligned_code_size_ = RoundUp(code_size, aligned_shift);
  // sort the ro_sections
  std::sort(ro_sections_.begin(), ro_sections_.end(),
            [](const Section* lhs, const Section* rhs) {
              if (lhs->alignment >= rhs->alignment) return true;
              return false;
            });
  return true;
}

size_t ConstantPoolResolver::ResolveOffset(unsigned offset,
                                           unsigned sym_index,
                                           unsigned addend,
                                           bool is_64) {
  size_t result = addend + aligned_code_size_;
  EMASSERT(is_64);
  unsigned sid = GetSectionId64(sym_index);
  for (const auto& s : ro_sections_) {
    if (s->id != sid) {
      result += s->bb->size();
      continue;
    }
    return result - offset;
  }
  UNREACHABLE();
}

unsigned ConstantPoolResolver::GetSectionId64(unsigned sym_index) {
  const Elf64_Sym* sym =
      reinterpret_cast<const Elf64_Sym*>(symbol_section_->bb->data());
  sym += sym_index;
#define ELF_ST_TYPE(x) (((unsigned int)x) & 0xf)
  EMASSERT(ELF_ST_TYPE(sym->st_info) == STT_SECTION);
  return sym->st_shndx;
}

void ConstantPoolResolver::EmitCP(compiler::Assembler& assembler) {
  // FIXME: need to reconsider x86 platform.
  while (assembler.CodeSize() != static_cast<intptr_t>(aligned_code_size_))
    assembler.Breakpoint();
  for (const auto& s : ro_sections_) {
    assembler.EmitRange(s->bb->data(), s->bb->size());
  }
}
}  // namespace

struct CodeAssembler::ArchImpl {
  ConstantPoolResolver constant_pool_resolver_;
  bool will_emit_cp_;
};

void CodeAssembler::PrepareLoadCPAction() {
  // Find relocation entries.
  const ByteBuffer* rela_buffer = compiler_state().FindByteBuffer(".rela.text");
  if (!rela_buffer) {
    arch_impl().will_emit_cp_ = false;
    return;
  }

  arch_impl().will_emit_cp_ = true;
  bool prepared =
      arch_impl().constant_pool_resolver_.Prepare(compiler_state(), code_size_);
  (void)prepared;
  EMASSERT(prepared);

  const Elf64_Rela* rela =
      reinterpret_cast<const Elf64_Rela*>(rela_buffer->data());
  size_t count = rela_buffer->size() / sizeof(Elf64_Rela);
  for (size_t i = 0; i < count; ++i, ++rela) {
    int rtype = ELF64_R_TYPE(rela->r_info);
    size_t offset = rela->r_offset;
    switch (rtype) {
      case R_AARCH64_LD_PREL_LO19: {
        auto func = [this, offset, rela] {
          const uint32_t* pc =
              reinterpret_cast<const uint32_t*>(code_start_ + offset);
          uint32_t instr = *pc;
          const uint32_t imm19_mask = ((1U << 19) - 1U) << 5U;
          size_t new_offset = arch_impl().constant_pool_resolver_.ResolveOffset(
              offset, ELF64_R_SYM(rela->r_info), rela->r_addend, true);

          size_t new_imm19 = new_offset >> 2;
          EMASSERT(new_imm19 == (new_imm19 & ((1U << 19U) - 1U)));
          instr &= ~imm19_mask;
          instr |= new_imm19 << 5U;
          assembler().EmitRange(&instr, 4);
          return 4;
        };
        AddAction(offset, WrapAction(func));
        break;
      }
      case R_AARCH64_ADR_PREL_LO21: {
        auto func = [this, offset, rela] {
          const uint32_t* pc =
              reinterpret_cast<const uint32_t*>(code_start_ + offset);
          uint32_t instr = *pc;
          size_t new_offset = arch_impl().constant_pool_resolver_.ResolveOffset(
              offset, ELF64_R_SYM(rela->r_info), rela->r_addend, true);
          EMASSERT(new_offset == (new_offset & ((1U << 21U) - 1U)));
          size_t new_offset_lo = new_offset & ((1U << 2) - 1U);
          size_t new_offset_hi = new_offset >> 2U;
          instr |= (new_offset_lo << 29) | (new_offset_hi << 5);
          assembler().EmitRange(&instr, 4);
          return 4;
        };
        AddAction(offset, WrapAction(func));
        break;
      }
      default:
        continue;
    }
  }
}

void CodeAssembler::EmitCP() {
  if (!arch_impl().will_emit_cp_) return;
  arch_impl().constant_pool_resolver_.EmitCP(assembler());
}

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
