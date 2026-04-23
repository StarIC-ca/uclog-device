// © 2024 Unit Circle Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// See: https://interrupt.memfault.com/blog/cortex-m-fault-debug
// https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortex-m4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf

#include <stdint.h>
#include <log.h>
#include <zephyr/kernel.h>

#define SCB_UFSR  (SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk)
#define SCB_BFSR  (SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk)
#define SCB_MMFSR (SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk)

#ifndef EXC_RETURN_PREFIX
  #define EXC_RETURN_PREFIX  (0xff000000)
#endif
#define EXC_RETURN_NOFPU   (0x00000010)
#define EXC_RETURN_THREAD  (0x00000008)
#define EXC_RETURN_PSP     (0x00000004)

typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  // Only if floating point HW enabled
  uint32_t s[16];
  uint32_t fpsrc;
  uint32_t undefined;
#endif
} stack_t;

static void memmanage_decode(bool from_hardfault) {
  log_panic_();
  LOG_ERROR("MemManage");
  if (SCB->CFSR & SCB_CFSR_MSTKERR_Msk) {
    LOG_ERROR("  Stacking access error");
  }
  else if (SCB->CFSR & SCB_CFSR_MUNSTKERR_Msk) {
    LOG_ERROR("  Unstacking access error");
  }
  else if (SCB->CFSR & SCB_CFSR_DACCVIOL_Msk) {
    uint32_t mmfar = SCB->MMFAR;
    __DMB();
    if (SCB->CFSR & SCB_CFSR_MMARVALID_Msk) {
      LOG_ERROR("  Data access error: 0x%08x", mmfar);
      if (from_hardfault) SCB->CFSR &= ~SCB_CFSR_MMARVALID_Msk;
    }
    else {
      LOG_ERROR("  Data access error");
    }
  }
  else if (SCB->CFSR & SCB_CFSR_IACCVIOL_Msk) {
    LOG_ERROR("  Instruction access violation");
  }
  else if (SCB->CFSR & SCB_CFSR_MLSPERR_Msk) {
    LOG_ERROR("  Floating-point lazy state preservation access error");
  }
  SCB->CFSR |= SCB_CFSR_MEMFAULTSR_Msk;
}

static void busfault_decode(bool from_hardfault) {
  LOG_ERROR("BusFault");
  if (SCB->CFSR & SCB_CFSR_STKERR_Msk) {
    LOG_ERROR("  Stacking bus error");
  }
  else if (SCB->CFSR & SCB_CFSR_UNSTKERR_Msk) {
    LOG_ERROR("  Unstacking bus error");
  }
  else if (SCB->CFSR & SCB_CFSR_PRECISERR_Msk) {
    uint32_t bfar = SCB->BFAR;
    __DMB();
    if (SCB->CFSR & SCB_CFSR_BFARVALID_Msk) {
      LOG_ERROR("  Precise data bus error: 0x%08x", bfar);
      if (from_hardfault) SCB->CFSR &= ~SCB_CFSR_BFARVALID_Msk;
    }
    else {
      LOG_ERROR("  Precise data bus error");
    }
  }
  else if (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk) {
      LOG_ERROR("  Imprecise data bus error");
    }
  else if (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk) {
    LOG_ERROR("  Imprecise data bus error");
  }
  else if (SCB->CFSR & SCB_CFSR_IBUSERR_Msk) {
    LOG_ERROR("  Instruction bus error");
  }
  else if (SCB->CFSR & SCB_CFSR_LSPERR_Msk) {
    LOG_ERROR("  Floating point lazy state preservation bus error");
  }
  SCB->CFSR |= SCB_CFSR_BUSFAULTSR_Msk;
}

static void usagefault_decode(void) {
  LOG_ERROR("UsageFault");
  if (SCB->CFSR & SCB_CFSR_DIVBYZERO_Msk) LOG_ERROR("  Divide by 0");
  if (SCB->CFSR & SCB_CFSR_UNALIGNED_Msk) LOG_ERROR("  Unaligned access");
  if (SCB->CFSR & SCB_CFSR_NOCP_Msk)      LOG_ERROR("  No coprocessor");
  if (SCB->CFSR & SCB_CFSR_INVPC_Msk)     LOG_ERROR("  Invalid PC");
  if (SCB->CFSR & SCB_CFSR_INVSTATE_Msk)  LOG_ERROR("  Invalid State");
  if (SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk)LOG_ERROR("  Undefined instruction");
  SCB->CFSR |= SCB_CFSR_USGFAULTSR_Msk;
}

static void hardfault_decode(void) {
  LOG_ERROR("HardFault");
  if (SCB->HFSR & SCB_HFSR_VECTTBL_Msk) {
    LOG_ERROR("  Bus fault on vector table read");
  }
  else if (SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk) {
    LOG_ERROR("  Debug event");
  }
  else if (SCB->HFSR & SCB_HFSR_FORCED_Msk) {
    LOG_ERROR("  Fault escalation...");
    if (SCB_MMFSR) {
      memmanage_decode(true);
    }
    else if (SCB_BFSR) {
      busfault_decode(true);
    }
    else if (SCB_UFSR) {
      usagefault_decode();
    }
  }
}

void fault_handler(const stack_t *stack, uint32_t lr) {
  uint32_t cfsr = SCB->CFSR;

  log_panic_();

  if (stack) {
    uint32_t fault = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
    switch (fault) {
      case 3:  hardfault_decode(); break;
      case 4:  memmanage_decode(false); break;
      case 5:  busfault_decode(false); break;
      case 6:  usagefault_decode(); break;
      default: LOG_ERROR("  unhandled exception type: %u", fault); break;
    }

    // Dump the "known stack"
    LOG_ERROR("  r0:   %08x r1:   %08x r2:   %08x r3:   %08x",
        stack->r0, stack->r1, stack->r2, stack->r3);
    LOG_ERROR("  r12:  %08x lr:   %08x pc:   %08x psr:  %08x",
        stack->r12, stack->lr, stack->pc, stack->psr);
    LOG_ERROR("  cfsr: %08x hfsr: %08x dfsr: %08x afsr: %08x",
        cfsr, SCB->HFSR, SCB->DFSR, SCB->AFSR);
    LOG_ERROR("lr_cur: %08x sp:   %08x", lr, (uint32_t) stack);

    // Skip what has already been printed
    uint32_t *stack_ = ((uint32_t*) stack) + 8;
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    if ((lr & EXC_RETURN_NOFPU) == 0) {
      __asm volatile("vsub.f32 s16,s16,s16": : ); // Force stacking of FPU regs
      __DSB();
      __ISB();
      LOG_ERROR("  s0:   %08x s1:   %08x s2:   %08x s3:   %08x",
          stack->s[0], stack->s[1], stack->s[2], stack->s[3]);
      LOG_ERROR("  s4:   %08x s5:   %08x s6:   %08x s7:   %08x",
          stack->s[4], stack->s[5], stack->s[6], stack->s[7]);
      LOG_ERROR("  s8:   %08x s9:   %08x s10:  %08x s11:  %08x",
          stack->s[8], stack->s[9], stack->s[10], stack->s[11]);
      LOG_ERROR("  s12:  %08x s13:  %08x s14:  %08x s15:  %08x",
          stack->s[12], stack->s[13], stack->s[14], stack->s[15]);
      LOG_ERROR("fpfrc:  %08x", stack->fpsrc);
      stack_ += 18; // Skip printed FPU regs
    }
#endif

    // Dump some of the stack in the hope that we can unwind after the fact
    extern uint32_t __kernel_ram_end[];
    uint32_t n = __kernel_ram_end - stack_;
    if (n > 32) n = 32;
    while (n > 0) {
      size_t nn = n > 4 ? 4 : n;
      LOG_MEM_ERROR("stack:", stack_, nn * sizeof(uint32_t));
      stack_ += nn;
      n -= nn;
    }
  }
  else {
    LOG_ERROR("Invalid EXC_RETURN value %08x", lr);
  }

  // If debugger connected issue breakpoint
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0) __BKPT(0);
  NVIC_SystemReset();
}

#if 0
#define HANDLER(x) \
__attribute__((naked, noreturn)) void x(void) { \
  /* Setup r0 with start point based on lr bit 2 (EXC_RETURN_PSP) */ \
  /* Then call C version to extract stack and various fault registers */ \
  __asm volatile ( \
    " tst lr, #4      \n" \
    " ite eq          \n" \
    " mrseq r0, msp   \n" \
    " mrsne r0, psp   \n" \
    " mov r1, lr   \n" \
    " b fault_handler \n" \
  ); \
}

HANDLER(HardFault_Handler)
HANDLER(MemManage_Handler)
HANDLER(BusFault_Handler)
HANDLER(UsageFault_Handler)

void fault_init(void) {
  SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
             |  SCB_SHCSR_BUSFAULTENA_Msk
             |  SCB_SHCSR_MEMFAULTENA_Msk;
}

void abort(void) {
  LOG_FATAL("abort called");
  for(;;);
}
#else

#if 0
void z_arm_fault(uint32_t msp, uint32_t psp, uint32_t exc_return,
	         void *callee_regs) {
  (void) callee_regs;

  (void) arch_irq_lock();
  if ((exc_return & EXC_RETURN_PREFIX) == EXC_RETURN_PREFIX) {
    stack_t* stack = (stack_t*) ((exc_return & EXC_RETURN_PSP) ? psp : msp);
    fault_handler(stack, exc_return);
  }
  else {
    fault_handler(NULL, exc_return);
  }
}
#endif

static const char *reason_to_str(unsigned int reason)
{
  switch ((enum k_fatal_error_reason)reason) { //-V2531 //-V2520
  case K_ERR_CPU_EXCEPTION:
    return "CPU exception";
  case K_ERR_SPURIOUS_IRQ:
    return "Unhandled interrupt";
  case K_ERR_STACK_CHK_FAIL:
    return "Stack overflow";
  case K_ERR_KERNEL_OOPS:
    return "Kernel oops";
  case K_ERR_KERNEL_PANIC:
    return "Kernel panic";
  default:
    return "Unknown error";
  }
}

void k_sys_fatal_error_handler(unsigned int reason,
				      const struct arch_esf *esf) {
  log_panic_();

  LOG_ERROR(">>> ZEPHYR FATAL ERROR %u: %s", reason,
    reason_to_str(reason));

  const stack_t* stack = (const stack_t*)esf; //-V1027 //-V2545 converting from struct arch_esf to stack_t, which contain the same members.
  fault_handler(stack, 0);
}

#ifdef CONFIG_BT_CTLR_ASSERT_HANDLER
void bt_ctlr_assert_handle(char *file, uint32_t line); // No public header with definition, so add one here to make MISRA happy

void bt_ctlr_assert_handle(char *file, uint32_t line) {
  log_panic_();

  LOG_ERROR(">>> SoftDevice Controller ASSERT: %s, %u", file, line);

  // If debugger connected issue breakpoint
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0) __BKPT(0);
  NVIC_SystemReset();
}
#endif

#ifdef CONFIG_MPSL_ASSERT_HANDLER
#include <mpsl/mpsl_assert.h>

void mpsl_assert_handle(char *file, uint32_t line) {
  log_panic_();

  LOG_ERROR(">>> MPSL ASSERT: %s, %u", file, line);

  // If debugger connected issue breakpoint
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0) __BKPT(0);
  NVIC_SystemReset();
}
#endif

#endif
