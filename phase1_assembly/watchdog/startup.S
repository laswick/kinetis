#define __ASM_SRC__
#include "arm_cm4.h"
#include "k60n512.h"

    .syntax unified
    .thumb
    .section .flash_config
flash_config_field:
    .long   0x01234567      /* Backdoor key                 */
    .long   0x89abcdef
    .long   0xffffffff      /* Program flash protection      */
    .byte   0xfe            /* Flash security byte           */
    .byte   0xff            /* Flash nonvolailte option byte */
    .byte   0xff            /* EEPROM protection byte        */
    .byte   0xff            /* Data flash protection byte    */

    .extern wdogDisable

    .text
    .align 2
    .globl __start_s
    .thumb_func
    .type  __start_s, %function
__start_s:
    bl wdogDisable

    /* Set Stack Pointer */
    ldr r1,=_bootstack_top
    mov sp,r1


__init_break:       /* First available breakpoint after reset for GDB */

configure_clock:
    /* Switch to ext 50MHz clk and adjust PLL for 96MHz core clock */
    /* Switch Clock Generator (MCG) to FLL Bypassed External (FBE) Mode */
    ldr  r1,=MCG_C2
    mov  r0,#0
    strb r0, [r1]

    ldr  r1,=MCG_C1
    mov  r0,#0x98        /* CLKS(2) | FRDIV(3) - External Ref Clk Div by 8 */
    strb r0, [r1]

mcg_s_irefst_wait:
    ldr  r1,=MCG_S
    ldrb r0, [r1]
    and  r0, r0,#0x10       /* IREFST - FLL Src is internal Reference Clk */
    beq  mcg_s_irefst_wait  /* Wait for IREFST to deassert */

mcg_s_clkst_wait:
    ldr  r1,=MCG_S
    ldrb r2, [r1]
    and  r0, r2, #0x0c      /* CLKST Mask */
    cmp  r0, #0x08          /* CLKST == 0b10 Externel Ref Clk */
    bne  mcg_s_clkst_wait   /* Wait for CLKST to equal 0b10 */

    /* Now in FBE Mode. Transition to PLL Bypassed External (PBE) */

    /* Divide down 50MHz clk by 25 to achieve required 2MHz for PLL */
    ldr  r1,=MCG_C5
    mov  r0,#0x18           /* PRDIV = 0b11000 - Divide by 25 */
    strb r0,[r1]

    /* Configure VCO to multiply PLL Clk by 25 to achieve 50MHz
       Can't seem to exceed this speed without locking issues */
    ldr  r1,=MCG_C6
    mov  r0,#0x41           /* PLLS | VDIV(0x1) - Enable PLL, VCO = 25x */
    strb r0,[r1]

mcg_s_pllst_wait:
    ldr  r1,=MCG_S
    ldrb r0,[r1]
    and  r0,r0,#0x20        /* PLLST - Source of PLL is PLL Clock */
    bne  mcg_s_pllst_wait   /* Wait for PLLST to assert */

mcg_s_lock_wait:
    ldr  r1,=MCG_S
    ldrb r0,[r1]
    and  r0,r0,#0x40        /* LOCK - PLL locked */
    bne  mcg_s_lock_wait    /* Wait for LOCK to assert */

    /* Now in PBE Mode. Transition to PLL Engaged External (PEE) */
    ldr  r1,=MCG_C1
    mov  r0, #0x0           /* Select PLL output as clock source */
    strb r0, [r1]

mcg_s_clkst_wait_2:
    ldr  r1,=MCG_S
    ldrb r2,[r1]
    and  r0, r2, #0x0c      /* CLKST Mask */
    cmp  r0, #0x0c          /* CLKST == 0b11 PLL Output Clk */
    bne  mcg_s_clkst_wait_2 /* Wait for CLKST to equal 0b11 */

    ldr r0,=main
    bx  r0

    .end
