/*******************************************************************************
*
* watchDog.c
*
* Andrew Krajewski
*
* Low level driver for the Kinetis Watchdog module.
*
* API: watchDogInit(), watchDogKick(), watchDogDisable(),
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
#define WDOG_TMROUTH    0x40052010
#define WDOG_TMROUTL    0x40052012
#define WDOG_RSTCNT     0x40052014
#define WDOG_PRESC      0x40052016

typedef struct {
    uint16_t stCtrlH;
    uint16_t stCtrlL;
    uint16_t toValH;
    uint16_t toValL;
    uint16_t winH;
    uint16_t winL;
    uint16_t refresh;
    uint16_t unlock;
    uint16_t tmrOutH;
    uint16_t tmrOutL;
    uint16_t rstCnt;
    uint16_t presc;
} watchDog_t; /* Mem Mapped registers */
static volatile watchDog_t *wdPtr = (volatile watchDog_t *) 0x40052000;

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
void watchDogUnlock()
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
#if 1
    /* Write the first unlock word */
    /* Write the second unlock word within 20 bus clock cycles */
    asm volatile("\n\
        ldr  r1, =0x4005200E @ WDOG_UNLOCK\n\
        ldr  r0, =0xC520 @ WDOG_UNLOCK_KEY_1\n\
        strh r0, [r1]\n\
        ldr  r0, =0xD928 @ WDOG_UNLOCK_KEY_2\n\
        strh r0, [r1]\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1" ); /* Specify which registers we destroy */
#else
    wdPtr->unlock = WDOG_UNLOCK_KEY_1;
    wdPtr->unlock = WDOG_UNLOCK_KEY_2;
#endif
    /* NOTE: Need to wait one clock cycle before updating any registers */
    int32_t test = 5;
    test += 1;
}

/*******************************************************************************
* watchDogInit
*******************************************************************************/
void watchDogInit(const watchDogConfig_t *wdCfgPtr)
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
#if 0
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
        ldr  r2,=0x40052006 @ WDOG_TOVALL\n\
        ldr  r0,=0x14c4b4c\n\
        strh r0,[r2]\n\
        ldr  r2,=0x40052004 @ WDOG_TOVALH\n\
        lsr  r0,r0,#16\n\
        strh r0,[r2]\n\
        ldr  r2,=0x40052000 @ WDOG_STCTRLH\n\
        ldr  r0,=0x01D3 @ STNDBYEN|WAITEN|STOPEN|ALLOWUPDATE|CLKSRC|WDOGEN \n\
        strh r0,[r2]\n\
        ldr  r2,=0x40052016 @ WDOG_PRESC\n\
        ldr  r0,=#0\n\
        strh r0,[r2]\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1", "r2" ); /* Specify which registers we destroy */
#else
//  assert(wdCfgPtr->window < wdCfgPtr->timeout);

    wdPtr->toValL  = wdCfgPtr->timeout;
    wdPtr->toValH  = wdCfgPtr->timeout >> 16;
    wdPtr->stCtrlH = wdCfgPtr->stCtrlFlags;
    wdPtr->presc   = wdCfgPtr->prescaler;
//  wdPtr->winL    = wdCfgPtr->window;
//  wdPtr->winH    = wdCfgPtr->window >> 16;

    if (wdCfgPtr->stCtrlFlags & WDOG_IRQRSTEN) {
        /* TODO: If anyone cares */
    }
#if 0
    if (wdCfgPtr->stCtrlFlags & WDOG_TEST) {
        /* TODO: If anyone cares */
    }
#endif
    /* TODO: Alt clock to LPO Oscillator */
#endif
}

static void interruptDisable()
{
    asm volatile("\n\
        cpsid i\n\
        " :
        /* No output */ :
        /* No input  */ :
        ); /* Specify which registers we destroy */
}
static void interruptEnable()
{
    asm volatile("\n\
        cpsie i\n\
        " :
        /* No output */ :
        /* No input  */ :
        ); /* Specify which registers we destroy */
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

#if 0
    asm volatile("\n\
        cpsid i\n\
        ldr  r1,=0x4005200C @ WDOG_REFRESH\n\
        ldr  r0,=0xA602 @ WDOG_REFRESH_KEY_1\n\
        strh r0,[r1]\n\
        ldr  r0,=0xB480 @ WDOG_REFRESH_KEY_2\n\
        strh r0,[r1]\n\
        cpsie i\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1" ); /* Specify which registers we destroy */
#else
    interruptDisable();
    wdPtr->refresh = WDOG_REFRESH_KEY_1;
    wdPtr->refresh = WDOG_REFRESH_KEY_2;
    interruptEnable();
#endif
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
#if 0
    asm volatile("\n\
        ldr  r1,=0x40052000 @ WDOG_STCTRLH\n\
        ldr  r0,=0x01D2 @ STNDBYEN | WAITEN | STOPEN | ALLOWUPDATE | CLKSRC \n\
        strh r0,[r1]\n\
        " :
        /* No output */ :
        /* No input  */ :
        "r0", "r1" ); /* Specify which registers we destroy */
#else
    wdPtr->stCtrlH = WDOG_STNDBYEN | WDOG_WAITEN | WDOG_STOPEN
                   | WDOG_ALLOWUPDATE;
#endif
}
