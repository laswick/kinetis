/*******************************************************************************
*
* watchDog.c
*
* Andrew Krajewski
*
* Low level driver for the Kinetis Watchdog module.
*
* API: watchDogConfig(), watchDogInit(), watchDogKick(), watchDogDisable(),
*      watchDogEnableInterrupt()
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

/* Watchdog Timer (WDOG) Module (s23) */
#define WDOG_STCTRLH    0x40052000
#define WDOG_STCTRLL    0x40052002
#define WDOG_TOVALH     0x40052004
#define WDOG_TOVALL     0x40052006
#define WDOG_WINH       0x40052008
#define WDOG_WINL       0x4005200A
#define WDOG_REFRESH    0x4005200C
#define WDOG_UNLOCK     0x4005200E
#define WDOG_TRMOUTH    0x40052010
#define WDOG_TMROUTL    0x40052012
#define WDOG_RSTCNT     0x40052014
#define WDOG_PRESC      0x40052016

#define WDOG_UNLOCK_KEY_1  0xC520
#define WDOG_UNLOCK_KEY_2  0xD928
#define WDOG_REFRESH_KEY_1 0xA602
#define WDOG_REFRESH_KEY_2 0xB480

/*******************************************************************************
*
* watchDogUnlock
*
* This routine unlocks the watchDog
*
* RETURNS: Nothing
*
*******************************************************************************/
static void watchDogUnlock()
{
#if 0
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
#endif
#if 0
    const uint32_t codeAddr = WDOG_UNLOCK;
    uint16_t key1 = WDOG_UNLOCK_KEY_1;
    uint16_t key2 = WDOG_UNLOCK_KEY_1;
    /* Write the first unlock word */
    asm volatile  ("strh %[addr], [%[value]]"
            : : [addr] "r" (codeAddr), [value] "r" (key1));

    /* Write the second unlock word within 20 bus clock cycles */
    asm volatile  ("strh %[addr], [%[value]]"
            : : [addr] "r" (codeAddr), [value] "r" (key2));
#else
    /* Write the first unlock word */
    /* Write the second unlock word within 20 bus clock cycles */
    asm volatile("\n\
        ldr  r1, =0x4005200E\n\
        ldr  r0, =0xC520\n\
        strh r0, [r1]\n\
        ldr  r0, =0xD928\n\
        strh r0, [r1]\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1" ); /* Specify which registers we destroy */
#endif
}

/*******************************************************************************
*
* watchDogConfig
*
* This routine configures the provided ?????. All available config
* options are in the WatchDog section in hardware.h.
*
* RETURNS: Nothing
*
*******************************************************************************/
void watchDogConfig()
{
    /* clock pre-scaler */
}

/*******************************************************************************
* watchDogInit
*******************************************************************************/
void watchDogInit()
{
#if 0
    push { r0, r1, r4, lr }

    bl   wdogUnlock
    /* Write the timeout value directly. Clock source is configured as LPO
     * oscillator which operates at 1KHz (s5.7.2) */
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
#endif

    watchDogUnlock();

    /* Write the timeout value directly. Clock source is configured as LPO
     * oscillator which operates at 1KHz (s5.7.2) */
/* NOTE: Changed from r4 to r2 as:
 * GCC allows you to destroy r0,r1,r2,r3, & r12 in your own stand-alone
 * assembly code procedures. All other registers must be preserved.
 * In this aspect, GCC follows the APCS (Arm Procedure Call Standard).
 * However, this does not apply to inline assembly code in C. For inline
 * assembly you may destroy registers but you must specify which you have
 * destroyed. */
    asm volatile("\n\
        ldr  r2,=0x40052006\n\
        strh r0,[r2]\n\
        ldr  r2,=0x40052004\n\
        lsr  r0,r0,#16\n\
        strh r0,[r2]\n\
        ldr  r2,=0x40052000\n\
        ldr  r0,=0x01D5\n\
        strh r0,[r2]\n\
        ldr  r2,=0x40052016\n\
        ldr  r0,=#0\n\
        strh r0,[r2]\n\
        ldr  r0,=0x00400000\n\
        ldr  r1,=0xE000E280\n\
        str  r0,[r1]\n\
        ldr  r1,=0xE000E100\n\
        str  r0,[r1]\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1", "r2" ); /* Specify which registers we destroy */
}

/*******************************************************************************
* watchDogKick
*******************************************************************************/
void watchDogKick()
{
#if 0
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
#endif
    asm volatile("\n\
        cpsid i\n\
        ldr  r1,=0x4005200C\n\
        ldr  r0,=0xA602\n\
        strh r0,[r1]\n\
        ldr  r0,=0xB480\n\
        strh r0,[r1]\n\
        cpsie i\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1" ); /* Specify which registers we destroy */
}

/*******************************************************************************
* watchDogDisable
*******************************************************************************/
void watchDogDisable()
{
#if 0
    push { r0, r1, lr }
    bl   wdogUnlock
    ldr  r1,=WDOG_STCTRLH
    ldr  r0,=0x01D2
    strh r0,[r1]
    pop  { r0, r1, lr }
    bx lr
#endif
    watchDogUnlock();

    asm volatile("\n\
        ldr  r1,=0x40052000\n\
        ldr  r0,=0x01D2\n\
        strh r0,[r1]\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1" ); /* Specify which registers we destroy */
}

/*******************************************************************************
* watchDogEnableInterrupt()
*******************************************************************************/
void watchDhgEnableInterrupt()
{
#if 0
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
#endif
}

