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

/* NOTE: About porting stand-alone asm into inline asm.
 * GCC allows you to destroy r0,r1,r2,r3, & r12 in your own stand-alone
 * assembly code procedures. All other registers must be preserved.
 * In this aspect, GCC follows the APCS (Arm Procedure Call Standard).
 * However, this does not apply to inline assembly code in C. For inline
 * assembly you may destroy registers but you must specify which you have
 * destroyed. */
/* If you wish to build the inlined assembly for comparison to the C
 * as a whole or each individual section then ...*/
/* #define INLINE_ASM */

/* Watchdog Timer (WDOG) Module (s23) */
#if 0 /* Strictly for reference of inline assembly */
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
#endif

#define WDOG_UNLOCK_KEY_1  0xC520
#define WDOG_UNLOCK_KEY_2  0xD928
#define WDOG_REFRESH_KEY_1 0xA602
#define WDOG_REFRESH_KEY_2 0xB480

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
    /* Write the first unlock word */
    /* Write the second unlock word within 20 bus clock cycles */
#if defined(INLINE_ASM)
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
}

/*******************************************************************************
* watchDogInit
*******************************************************************************/
void watchDogInit(const watchDogConfig_t *wdCfgPtr)
{
    watchDogUnlock();

#if defined(INLINE_ASM)
    /* Write the timeout value directly. Clock source is configured as LPO
     * oscillator which operates at 1KHz (s5.7.2) */
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
    assert(wdCfgPtr->window < wdCfgPtr->timeout);

    wdPtr->toValL  = wdCfgPtr->timeout;
    wdPtr->toValH  = wdCfgPtr->timeout >> 16;
    wdPtr->stCtrlH = wdCfgPtr->stCtrlFlags;
    wdPtr->presc   = wdCfgPtr->prescaler;
    wdPtr->winL    = wdCfgPtr->window;
    wdPtr->winH    = wdCfgPtr->window >> 16;

    if (wdCfgPtr->stCtrlFlags & WDOG_IRQRSTEN) {
        /* TODO: If anyone cares let me know */
    }
    if (wdCfgPtr->stCtrlFlags & WDOG_TEST) {
        /* TODO: If anyone cares let me know */
    }
#endif
}

/* RFI: Could probably move these elsewhere for general consumption */
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
#if defined(INLINE_ASM)
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
    watchDogUnlock();

#if defined(INLINE_ASM)
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
