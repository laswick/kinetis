#define __ASM_SRC__
#include "arm_cm4.h"
#include "k60n512.h"

#define WDOG_UNLOCK_KEY_1  0xC520
#define WDOG_UNLOCK_KEY_2  0xD928
#define WDOG_REFRESH_KEY_1 0xA602
#define WDOG_REFRESH_KEY_2 0xB480

    .syntax unified
    .thumb
    .text

    .align 2
    .globl  wdogUnlock
    .thumb_func
    .type   wdogUnlock, %function
wdogUnlock:
    push { r0, r1 }
    ldr  r1, =WDOG_UNLOCK
    /* Write the first unlock word */
    ldr  r0, =WDOG_UNLOCK_KEY_1
    strh r0, [r1]
    /* Write the second unlock word within 20 bus clock cycles */
    ldr  r0, =WDOG_UNLOCK_KEY_2
    strh r0, [r1]
    pop  { r0, r1 }
    bx   lr

/*****************************************************************************
 * wdogDisable
 *      Disables the hardware watchdog
 *
 *      Inputs:
 *      <None>
 *      Outputs:
 *      <None>
 ****************************************************************************/
    .align 2
    .globl wdogDisable
    .thumb_func
    .type   wdogDisable, %function
wdogDisable:
    push { r0, r1, lr }
    bl   wdogUnlock
    ldr  r1,=WDOG_STCTRLH
    ldr  r0,=0x01D2
    strh r0,[r1]
    pop  { r0, r1, lr }
    bx lr

/*****************************************************************************
 * wdogPunch
 *      Punches the hardware watchdog
 *
 *      Inputs:
 *      <None>
 *      Outputs:
 *      <None>
 ****************************************************************************/
    .align 2
    .globl wdogPunch
    .thumb_func
    .type  wdogPunch, %function
wdogPunch:
    push { r0, r1, lr }
    cpsid i
    ldr  r1,=WDOG_REFRESH
    ldr  r0,=WDOG_REFRESH_KEY_1
    strh r0,[r1]
    ldr  r0,=WDOG_REFRESH_KEY_2
    strh r0,[r1]
    cpsie i
    pop { r0, r1, lr }
    bx lr

/*****************************************************************************
 * wdogInit
 *      Configures and starts the hardware watchdog module
 *
 *      Inputs:
 *      r0: Timeout Value in mSec
 *      Outputs:
 *      <None>
 ****************************************************************************/
    .align 2
    .globl wdogInit
    .thumb_func
    .type  wdogInit, %function
wdogInit:
    push { r0, r1, r4, lr }

    bl   wdogUnlock
    /*
     * Write the timeout value directly. Clock source is configured as LPO
     * oscillator which operates at 1KHz (s5.7.2)
     */
    ldr  r4,=WDOG_TOVALL
    strh r0,[r4]
    ldr  r4,=WDOG_TOVALH
    lsr  r0,r0,#16
    strh r0,[r4]
    ldr  r4,=WDOG_STCTRLH
    ldr  r0,=0x01D5 /* STNDBYEN | STOPEN | ALLOWUPDATE | IRQSTEN | WDOGEN */
    strh r0,[r4]
    ldr  r4,=WDOG_PRESC
    ldr  r0,=#0     /* No clock prescaler */
    strh r0,[r4]

    /*
     * Enable the watchdog IRQ in the NVIC module
     */
    ldr  r0,=BIT_22         /* IRQ 22 WDOG*/
    ldr  r1,=NVIC_ICPR0
    str  r0,[r1]
    ldr  r1,=NVIC_ISER0
    str  r0,[r1]

    pop  { r0, r1, r4, lr }
    bx lr



    .extern _wdog_debug    /* From Linker */

    /* TODO: This interrupt never seems to get triggered. Other people
             are citing the same problem but no errata data yet */
    .align 2
    .global __isr_wdog
    .thumb_func
    .type   __isr_wdog, %function
__isr_wdog:                 /* Save off state of registers to ram */
    ldr r8,=_wdog_debug
    mov r7,#0x5a5a5a5a
    str r7,[r8]!
    str r0,[r8]!
    str r1,[r8]!
    str r2,[r8]!
    str r3,[r8]!
    str r9,[r8]!
    str ip,[r8]!
    str sp,[r8]!
    str lr,[r8]!
    str r7,[r8]!
loop:                       /* Wait for watchdog to issue reset */
    b loop

    .end
