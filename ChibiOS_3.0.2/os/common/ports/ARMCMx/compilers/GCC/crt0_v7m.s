/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    crt0_v7m.s
 * @brief   Generic ARMv7-M (Cortex-M3/M4/M7) startup file for ChibiOS.
 *
 * @addtogroup ARMCMx_GCC_STARTUP_V7M
 * @{
 */

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

#if !defined(FALSE) || defined(__DOXYGEN__)
#define FALSE                               0
#endif

#if !defined(TRUE) || defined(__DOXYGEN__)
#define TRUE                                1
#endif

#define CONTROL_MODE_PRIVILEGED             0
#define CONTROL_MODE_UNPRIVILEGED           1
#define CONTROL_USE_MSP                     0
#define CONTROL_USE_PSP                     2
#define CONTROL_FPCA                        4

#define FPCCR_ASPEN                         (1 << 31)
#define FPCCR_LSPEN                         (1 << 30)

#define SCB_CPACR                           0xE000ED88
#define SCB_FPCCR                           0xE000EF34
#define SCB_FPDSCR                          0xE000EF3C

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   FPU initialization switch.
 */
#if !defined(CRT0_INIT_FPU) || defined(__DOXYGEN__)
#if defined(CORTEX_USE_FPU) || defined(__DOXYGEN__)
#define CRT0_INIT_FPU                       CORTEX_USE_FPU
#else
#define CRT0_INIT_FPU                       FALSE
#endif
#endif

/**
 * @brief   Control special register initialization value.
 * @details The system is setup to run in privileged mode using the PSP
 *          stack (dual stack mode).
 */
#if !defined(CRT0_CONTROL_INIT) || defined(__DOXYGEN__)
#define CRT0_CONTROL_INIT                   (CONTROL_USE_PSP |              \
                                             CONTROL_MODE_PRIVILEGED)
#endif

/**
 * @brief   Core initialization switch.
 */
#if !defined(CRT0_INIT_CORE) || defined(__DOXYGEN__)
#define CRT0_INIT_CORE                      TRUE
#endif

/**
 * @brief   Stack segments initialization switch.
 */
#if !defined(CRT0_STACKS_FILL_PATTERN) || defined(__DOXYGEN__)
#define CRT0_STACKS_FILL_PATTERN            0x55555555
#endif

/**
 * @brief   Stack segments initialization switch.
 */
#if !defined(CRT0_INIT_STACKS) || defined(__DOXYGEN__)
#define CRT0_INIT_STACKS                    TRUE
#endif

/**
 * @brief   DATA segment initialization switch.
 */
#if !defined(CRT0_INIT_DATA) || defined(__DOXYGEN__)
#define CRT0_INIT_DATA                      TRUE
#endif

/**
 * @brief   BSS segment initialization switch.
 */
#if !defined(CRT0_INIT_BSS) || defined(__DOXYGEN__)
#define CRT0_INIT_BSS                       TRUE
#endif

/**
 * @brief   Constructors invocation switch.
 */
#if !defined(CRT0_CALL_CONSTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_CONSTRUCTORS              TRUE
#endif

/**
 * @brief   Destructors invocation switch.
 */
#if !defined(CRT0_CALL_DESTRUCTORS) || defined(__DOXYGEN__)
#define CRT0_CALL_DESTRUCTORS               TRUE
#endif

/**
 * @brief   FPU FPCCR register initialization value.
 * @note    Only used if @p CRT0_INIT_FPU is equal to @p TRUE.
 */
#if !defined(CRT0_FPCCR_INIT) || defined(__DOXYGEN__)
#define CRT0_FPCCR_INIT                     (FPCCR_ASPEN | FPCCR_LSPEN)
#endif

/**
 * @brief   CPACR register initialization value.
 * @note    Only used if @p CRT0_INIT_FPU is equal to @p TRUE.
 */
#if !defined(CRT0_CPACR_INIT) || defined(__DOXYGEN__)
#define CRT0_CPACR_INIT                     0x00F00000
#endif

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

#if !defined(__DOXYGEN__)

                .syntax unified       //use unified instructions for both ARM and THUMB modes
                .cpu    cortex-m3     //select target processor(-mcpu)
#if CRT0_INIT_FPU == TRUE
                .fpu    fpv4-sp-d16   //select the floating-point unit to assemble for(-mfpu): version4, single point, with 16 double-point registers
#else
                .fpu    softvfp
#endif

                .thumb                //following statements use thumb instruction set
                .text                 //following statements are assembled into .text 0 subsection

/*
 * Reset handler.
 */
                .align  2             //align to 2 bytes
                .thumb_func           //the next symbol(Reset_Handler) is a thumb function
                .global Reset_Handler //make Reset_Handler visible to the linker(ld)
Reset_Handler:
                /* Interrupts are globally masked initially.*/
                //cpsid="Change Processor State: Interrupt Disable" and i="Primask", this statement disables all interrupts including NMI.
                cpsid   i

                /* PSP stack pointers initialization.*/
                ldr     r0, =__process_stack_end__  //give __process_stack_end__ to generic reg r0, and then:
                msr     PSP, r0                     //give it to special reg PSP, to be initial process stack pointer

#if CRT0_INIT_FPU == TRUE
                /* FPU FPCCR initialization.*/
                /* Set FPCCR("Floting-Point Context Control Register") to [ FPCCR_ASPEN | FPCCR_LSPEN ], in order to enable "Lazy Stacking" feature.
                  FPCCR_ASPEN means "Automatic State Preservation Enable",
                  FPCCR_LSPEN means "Lazy State Preservation Enable"
                  
                  "Lazy Stacking":
                  - when exception entry/exit occurs, FPU regs will be preserved/restored, except for two situations delow:
                  - 1. the exception entering will not use FPU;
                  - 2. the exception exiting did not use FPU ever.
                */
                movw    r0, #CRT0_FPCCR_INIT & 0xFFFF   //move a 16-bit value to lower halfWord of r0 (r0[15:0])
                movt    r0, #CRT0_FPCCR_INIT >> 16      //move a 16-bit value to higher(Top) halfword of r0 (r0[31:16])
                movw    r1, #SCB_FPCCR & 0xFFFF
                movt    r1, #SCB_FPCCR >> 16
                str     r0, [r1]  //str Rt, [Rn]: store one word value Rt to memory [Rn].
                dsb     //Data Synchronization Barrier: do not execute instructions below, until instructions above be executed completely.
                isb     //Instruction Synchronization Barrier: it is used after the DSB to ensure the processor pipeline is flushed and subsequent instructions are re-fetched.

                /* CPACR initialization.*/
                /* Set CPACR("CoProcessor Access Control Register") to [ CP11 | CP10 ] = [ 0b1111 ], in order to set FPU (coprocessor) to be "full access".

                  0b00: Access denied. Any attempted access generates a NOCP UsageFault.
                  0b01: Privileged access only. An unprivileged access generates a NOCP fault.
                  0b10: Reserved. The result of any access is Unpredictable.
                  0b11: Full access (Privileged mode and user mode).

                  On the Cortex-M4, the FPU is defined as co-processor 10 and 11. Since there is no other co-processor, 
                  only CP10 and CP11 are available and both are for the FPU, and they should be writen the same value.
                */
                movw    r0, #CRT0_CPACR_INIT & 0xFFFF
                movt    r0, #CRT0_CPACR_INIT >> 16
                movw    r1, #SCB_CPACR & 0xFFFF
                movt    r1, #SCB_CPACR >> 16
                str     r0, [r1]
                dsb
                isb

                /* FPU FPSCR initially cleared.*/
                mov     r0, #0
                vmsr    FPSCR, r0   //vmsr FPSCR, r0: move to Floating-Point Status Control Register from r0 (Arm Core register).

                /* FPU FPDSCR initially cleared.*/
                movw    r1, #SCB_FPDSCR & 0xFFFF
                movt    r1, #SCB_FPDSCR >> 16
                str     r0, [r1]    //Store r0 = 0x0 to memory [r1] = SCB_FPDSCR

                /* Enforcing FPCA bit in the CONTROL register.*/
                
                /* Move value [ FPCA | SPSEL | ~nPRIV ] to r0 (ready to move to CONTROL register), 
                  and update APSR (Application Program Status Register) with N/C/Z bits in it.
                  (movs = MOVe with Setting flags in APSR)

                  FPCA = Floating-Point Context Active. The Cortex-M4 uses this bit to determine whether 
                        to preserve floating-point state when processing an exception.
                  SPSEL = Stack Pointer SELection. 1 = PSP is the current stack pointer.
                  nPRIV = Thread mode privilege level. 0 = Privileged.
                */
                movs    r0, #CRT0_CONTROL_INIT | CONTROL_FPCA   

#else
                movs    r0, #CRT0_CONTROL_INIT
#endif

                /* CONTROL register initialization as configured.*/

                /* "After updating the CONTROL register with MSR
                  instruction, the ISB instruction should be used to
                  ensure the updated configuration is used for
                  subsequent operations."
                */
                msr     CONTROL, r0
                isb

#if CRT0_INIT_CORE == TRUE
                /* Core initialization.*/
                bl      __core_init //Actually do nothing.
#endif

                /* Early initialization.*/
                bl      __early_init  //Don't know which function will be executed?

#if CRT0_INIT_STACKS == TRUE
                ldr     r0, =CRT0_STACKS_FILL_PATTERN
                /* Main Stack initialization. Note, it assumes that the
                   stack size is a multiple of 4 so the linker file must
                   ensure this.*/
                ldr     r1, =__main_stack_base__
                ldr     r2, =__main_stack_end__
msloop:
                cmp     r1, r2
                itt     lo            //itt = "If-Then-True": there will be two following xxxlo instructions being executed when r1 is really lower than r2.
                strlo   r0, [r1], #4  //Initialize main stack: fill the memory from __main_stack_base__ to __main_stack_end__ with CRT0_STACKS_FILL_PATTERN = 0x55555555.
                blo     msloop

                /* Process Stack initialization. Note, it assumes that the
                   stack size is a multiple of 4 so the linker file must
                   ensure this.*/
                ldr     r1, =__process_stack_base__
                ldr     r2, =__process_stack_end__
psloop:
                cmp     r1, r2
                itt     lo
                strlo   r0, [r1], #4  //Initialize process stack.
                blo     psloop
#endif

#if CRT0_INIT_DATA == TRUE
                /* Data initialization. Note, it assumes that the DATA size
                  is a multiple of 4 so the linker file must ensure this.*/
                ldr     r1, =_textdata
                ldr     r2, =_data
                ldr     r3, =_edata
dloop:
                cmp     r2, r3
                ittt    lo
                ldrlo   r0, [r1], #4
                strlo   r0, [r2], #4    //Initialize data: transfer data from _textdata to _edata segment (But what do they mean?).
                blo     dloop
#endif

#if CRT0_INIT_BSS == TRUE
                /* BSS initialization. Note, it assumes that the DATA size
                  is a multiple of 4 so the linker file must ensure this.*/
                movs    r0, #0
                ldr     r1, =_bss_start
                ldr     r2, =_bss_end
bloop:
                cmp     r1, r2
                itt     lo
                strlo   r0, [r1], #4  //Initialize bss segment: to zeros.
                blo     bloop
#endif

                /* Late initialization..*/
                bl      __late_init   //Actually do nothing.

#if CRT0_CALL_CONSTRUCTORS == TRUE
                /* Constructors invocation.*/
                ldr     r4, =__init_array_start
                ldr     r5, =__init_array_end
initloop:
                cmp     r4, r5
                bge     endinitloop
                ldr     r1, [r4], #4
                /* blx = Branch and Link with eXchange (to arm instrucion).
                  Comparing with BL <label>, BLX <Rm> branches indirectly (means jump to somewhere not so obvious),
                  while BL <label> branches directly with an obvious indicating label.
                 */
                blx     r1        //Call every constructor functions listed in __init_array[].
                b       initloop
endinitloop:
#endif

                /* Main program invocation, r0 contains the returned value.*/
                bl      main

#if CRT0_CALL_DESTRUCTORS == TRUE
                /* Destructors invocation.*/
                ldr     r4, =__fini_array_start
                ldr     r5, =__fini_array_end
finiloop:
                cmp     r4, r5
                bge     endfiniloop
                ldr     r1, [r4], #4
                blx     r1
                b       finiloop
endfiniloop:
#endif

                /* Branching to the defined exit handler.*/
                b       __default_exit

#endif /* !defined(__DOXYGEN__) */

/** @} */
