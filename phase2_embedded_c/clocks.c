/*******************************************************************************
*
* clocks.c
*
* Jan Markowski
*
* Low level driver for the Kinetis Clock module.
*
* API:  clockSetDividers(), clockGetFreq()
*       
*       clockConfigMcgOut(), clockConfigMcgIr(), clockConfigMcgFf(), 
*       clockConfigMcgFll(), clockConfigMcgPll()
*       
*       clockConfigOsc(), clockConfigOsc32k(), clockConfigOscEr()
*
*       clockConfigEr32k(), clockConfigRtc(), clockConfigLpo()
*
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

typedef struct {
    volatile mcg_t *mcg;
    volatile osc_t *osc;
    volatile rtc_t *rtc;
} clockReg_t; 

static clockReg_t clock = {
    .mcg     = (volatile mcg_t *) MCG_BASE_ADDR,
    .osc     = (volatile osc_t *) OSC_BASE_ADDR,
    .rtc     = (volatile rtc_t *) RTC_BASE_ADDR,
};

typedef enum {
    MODE_FEI,                    /* [DEFAULT UPON RESET] FLL Engaged Internal */
    MODE_FEE,                                         /* FLL Engaged External */
    MODE_PEE,                                         /* PLL Engaged External */
    MODE_BLPI,                                 /* Bypassed Low Power Internal */
    MODE_BLPE,                                 /* Bypassed Low Power External */
    MODE_FBI,                                        /* FLL Bypassed Internal */
    MODE_FBE,                                        /* FLL Bypassed External */
    MODE_PBE,                                        /* PLL Bypassed External */
    MAX_MODES,
    MODE_STOP,        /* Stop mode. Entered whenever MCU enters a Stop state. */
} clockMode_t;

typedef enum {
    EXTERNAL_OSC_50MHZ,
    INTERNAL_32KHZ,
} freqSource_t;

typedef struct {
   clockMode_t     clockMode;
   uint8_t         divider;
   uint8_t         multiplier;
   freqSource_t    freqSource;
   uint32_t        clockHz;
} clockConfigParam_t;

/* 
 * Any additional configurations must be listed in clockConfig_t of hardware.h
 */
clockConfigParam_t clockConfigParam[MAX_MCG_CLOCK_OPTIONS] = {

    [MCG_PLL_EXTERNAL_100MHZ] = {
        .clockMode   = MODE_PEE,
        /* 
         * Note: For PLL, the received external clock must be 2 - 4MHz 
         *       Here, we receive 50MHz from the external source, and divide
         *       by 25 to get 2MHz.
         *
         *       ONLY external clock sources may be used for PLL.
         */
        .divider     = MCG_C5_PRDIV_MASK & 0x18,  /* divide 50MHz by 25 = 2MHz*/
        .multiplier  = MCG_C6_VDIV_MASK & 0x1A, /* multiply 2MHz by 50 =100MHz*/
        .freqSource  = EXTERNAL_OSC_50MHZ,        /* Must be external for PLL */
        .clockHz     = 100000000,        /* The resulting MCGOUTCLK frequency */
    },

    [MCG_FLL_INTERNAL_24MHZ] = {
        .clockMode   = MODE_FEI,
        .divider     = NULL,  /* Dividers are only for externally engaged (EE)*/
        /*
         * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
         * WARNING: Do not enter a clock multiplier that would exceed the system
         * clock capability of 100MHz (could damage FLL).
         * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
         */
        .multiplier  = (MCG_C4_DRST_DRS_MASK & 0x00 << 5) | /* Low freq range */
                       (MCG_C4_DMX32 & BIT_7),  /* FLL factor is 732 */
        .freqSource  = INTERNAL_32KHZ,
        .clockHz     = 23986176, /* 32.768 * 732 */
    },

};

typedef struct {
    clockMode_t   currentMode;
    clockMode_t   nextMode; 
    clockConfig_t clockConfig;
} mcgState_t; 

/* Initial value is FEI. This is entered upon reset. */
static mcgState_t mcgState; 

typedef struct {
    uint32_t    mcgClockFreq;
    divider_t   systemDiv;
    divider_t   busDiv;
    divider_t   flexBusDiv;
    divider_t   flashDiv;
} clockFreq_t;

static clockFreq_t clockFreq = {
    /*
     * On reset, the clock is defaulted to FEI mode, where MCGOUTCLK is derived
     * from the FLL clock. The reset clock source is the slow internal reference
     * clock (IRC) of 32 kHz. The default multiplication factor from
     * DMX32/DRST_DRS in MCG Control Register 4 is 640.
     * 32.768 kHz * 640 = 20480000 Hz.
     *
     * The FTFL_FOPT[LPBOOT] bit determines the clock divider values after reset
     * (different if you are resetting from a low power state, or a fast clock
     * state). (section 5.5.1)
     *
     * For the fast clock state, the dividers are 1/1/2/2 for the
     * system/bus/flexbus/flash.
     */
    .mcgClockFreq = SYSTEM_CLOCK_HZ_DFLT,
    .systemDiv    = SYSTEM_DIVIDER_DFLT,
    .busDiv       = BUS_DIVIDER_DFLT,
    .flexBusDiv   = FLEXBUS_DIVIDER_DFLT,
    .flashDiv     = FLASH_DIVIDER_DFLT,
};


static void fei2fee(clockConfig_t cc)
{
    /* 
     * The FRDIV selects the amount to divide down the external reference
     * clock for the FLL. 
     *
     * In our case, we have a 50MHz external reference clock. This frequency
     * must be divided down to fall in the range between 31.25 kHz and 39.0625
     * kHz. 
     *
     * Uno problemo: The largest divide factor available by FRDIV is 1024... And
     * 50MHz / 1024 = 48.3 kHz, which exceeds the 39.0625 kHz range. With our
     * setup, it doesn't seem that we can reach a FEE mode (and I don't want to
     * risk the tower to see what could happen if I did!)
     */
}

static void fei2pee(clockConfig_t cc) 
{
                                                    /* External crystal setup */
    /* Select the OSCCLK */
    SIM_SOPT2 &= ~SIM_SOPT2_MCGCLKSEL;

    /* 
     * Enabling the XTAL for 50MHz
     * RANGE=1, match the frequency of the crystal being used
     * HGO=1,   set for high gain operation (best against noise)
     * EREFS=1, enable the external oscillator
     */
    clock.mcg->c2 = (MCG_C2_RANGE_MASK & (0x1 << 4)) |
                    (MCG_C2_HGO) | (MCG_C2_EREFS);

                                                            /* Enter FBE mode */
    /*
     * CLKS=2,  select the external clock.
     *
     * FRDIV=3, Normally, the external clock has to be divided to keep the
     * resulting frequency in the 31.25 to 39.0625kHz range. However, in FBE
     * mode, this does not matter. It only matters when you try to enter an FLL
     * mode from FBE (here we are going to PEE)
     *
     * IREFS=0, select external reference clock and enable the external
     * oscillator.
     */
    clock.mcg->c1 = ((MCG_C1_CLKS_MASK & (0x2 << 6)) |
                    (MCG_C1_FRDIV_MASK & (0x3 << 3))) &
                    (~MCG_C1_IREFS);

                                                    /* Wait for status update */
    /* Wait for oscillator to initialize */
    while (!(clock.mcg->s & MCG_S_OSCINIT)) {}
    /* Wait for reference clock's to become the external reference */
    while (clock.mcg->s & MCG_S_IREFST) {}
    /* Wait for the indicator that MCGOTUCLK is fed by the external ref clock */
    while ((clock.mcg->s & MCG_S_CLKST_MASK) != (0x2 << 2)) {}

                                            /* Generate correct PLL frequency */
    clock.mcg->c5 &= 0xE0; /* clear bits */
    clock.mcg->c5 |= clockConfigParam[cc].divider;
    clock.mcg->c6 &= 0xE0;
    clock.mcg->c6 |= clockConfigParam[cc].multiplier;
                                                            /* Enter PBE mode */
    /* PLLS=1, select the PLL. */
    clock.mcg->c6 |= MCG_C6_PLLS;
     
                                                    /* Wait for status update */
    /* Wait for the PLL to be the clock source */
    while (!(clock.mcg->s & MCG_S_PLLST)) {}
    /* Wait until the PLL has acquired lock on the external frequency */
    while (!(clock.mcg->s & MCG_S_LOCK)) {}

                                                            /* Enter PEE mode */
    /* Select the output of the PLL */
    clock.mcg->c1 &= ~MCG_C1_CLKS_MASK;

                                                    /* Wait for status update */
    /* Wait until output of the PLL is selected */
    while ((clock.mcg->s & MCG_S_CLKST_MASK) != (0x3 << 2)) {}

    mcgState.currentMode = MODE_PEE;
}

static void fei2blpi(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void fei2blpe(clockConfig_t cc) 
{
                                                            /* Not configured */
}


static void fee2fei(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void fee2pee(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void fee2blpi(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void fee2blpe(clockConfig_t cc) 
{
                                                            /* Not configured */
}


static void pee2fei(clockConfig_t cc)
{

                                                            /* Enter PBE mode */
    clock.mcg->c1 = (MCG_C1_CLKS_MASK & (0x2 << 6)); /* Select external clock */

                                                    /* Wait for status update */
    while ((clock.mcg->s & MCG_S_CLKST_MASK) != (0x2 << 2)) {}

                                                            /* Enter FBE mode */
    /* 
     * With the FLL frequency valid, we can now clear the PLLS bit to select FLL
     */
    clock.mcg->c6 &= ~MCG_C6_PLLS;

                                                    /* Wait for status update */
    /* Wait until the current source is FLL */
    while (clock.mcg->s & MCG_S_PLLST) {}

                                            /* Generate correct FLL frequency */
    clock.mcg->c4 = clockConfigParam[cc].multiplier;

                                                            /* Enter FEI mode */
    clock.mcg->c1 = (MCG_C1_CLKS_MASK & (0x0 << 6)) |
                    (MCG_C1_IREFS);

                                                    /* Wait for status update */
    /* Wait for reference clock's to become the internal reference */
    while (!(clock.mcg->s & MCG_S_IREFST)) {}
    /* Wait until the output of the FLL is selected */
    while ((clock.mcg->s & MCG_S_CLKST_MASK) != (0x0 << 2)) {}

    mcgState.currentMode = MODE_FEI;
}

static void pee2fee(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void pee2blpi(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void pee2blpe(clockConfig_t cc) 
{
                                                            /* Not configured */
}


static void blpi2fei(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void blpi2fee(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void blpi2pee(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void blpi2blpe(clockConfig_t cc) 
{
                                                            /* Not configured */
}


static void blpe2fei(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void blpe2fee(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void blpe2pee(clockConfig_t cc) 
{
                                                            /* Not configured */
}

static void blpe2blpi(clockConfig_t cc) 
{
                                                            /* Not configured */
}

/*******************************************************************************
*
* clockSetDividers
*
* The clock dividers divide the MCGOUTCLK frequency to produce the resultant
* system, bus, flexbus, and flash clock frequencies. 
*
*******************************************************************************/
void clockSetDividers(divider_t systemDiv, divider_t busDiv, 
                                       divider_t flexBusDiv, divider_t flashDiv)
{
    int mcgClock = clockFreq.mcgClockFreq;

    /* 
     * The asserts are raised when the internal clock requirements (sec. 5.5)
     * are not met.
     */
    assert( (mcgClock / (systemDiv +1)) <= MAX_SYSTEM_FREQ);

    assert( (mcgClock / (busDiv    +1)) <= MAX_BUS_FREQ &&
            (mcgClock / (busDiv    +1)) <= (mcgClock / (systemDiv+1)));

    assert( (mcgClock / (flashDiv  +1)) <= MAX_FLASH_FREQ &&
            (mcgClock / (flashDiv  +1)) <= (mcgClock / (busDiv+1)));

    assert( (mcgClock / (flexBusDiv+1)) <= (mcgClock / (busDiv+1)));

    /* Save the new dividers */
    clockFreq.systemDiv  = systemDiv;
    clockFreq.busDiv     = busDiv;
    clockFreq.flexBusDiv = flexBusDiv;
    clockFreq.flashDiv   = flashDiv;

    /* Set the SIM clock dividers */
    SIM_CLKDIV1 = (systemDiv << 28)  | 
                  (busDiv << 24)     | 
                  (flexBusDiv << 20) |
                  (flashDiv << 16);
}

/*******************************************************************************
*
* clockGetFreq
*
* Grab the clock frequency for a particular clock source in Hz.
*
*******************************************************************************/
uint32_t clockGetFreq(clockSource_t cs)
{
    uint32_t clock;

    switch (cs) {
    case CLOCK_SYSTEM:
        clock = clockFreq.mcgClockFreq / (clockFreq.systemDiv + 1);
        break;
    case CLOCK_BUS:
        clock = clockFreq.mcgClockFreq / (clockFreq.busDiv + 1);
        break;
    case CLOCK_FLEXBUS:
        clock = clockFreq.mcgClockFreq / (clockFreq.flexBusDiv + 1);
        break;
    case CLOCK_FLASH:
        clock = clockFreq.mcgClockFreq / (clockFreq.flashDiv + 1);
        break;
    default:
        assert(0);
    }

    return clock;
}

/*******************************************************************************
*
* clockConfigMcgOut
*
* This configures the clock frequency for the SYSTEM/CORE, BUS, FLASH, and
* FLEXBUS. 
* 
* State Machine
*
* All modes can move to any other mode on the same line. 
*
*  RESET 
*      \___ FEI ____ FBI ____ BLPI  (Internal source branch)
*                |    
*                |  (External source branch)
*                |
*                |__ FEE
*                |    
*                |__ FBE ____ BLPE
*                         |
*                         |__ PBE ____ PEE
*
*        ____ STOP ____ (Entered when MCU enters stop mode, and returns to
*                        previous active mode when exits stop mode )
*
*******************************************************************************/
void clockConfigMcgOut(clockConfig_t clockConfig)
{

    /*
     * This is the jump table. Depending on the current state, and the desired
     * state, the appropriate function handler is called.
     */
    static void (* const jumpTable[5][5])(clockConfig_t cc) = { 
        {     NULL,  fei2fee,  fei2pee,  fei2blpi,  fei2blpe, },
        {  fee2fei,     NULL,  fee2pee,  fee2blpi,  fee2blpe, },
        {  pee2fei,  pee2fee,     NULL,  pee2blpi,  pee2blpe, },
        { blpi2fei, blpi2fee, blpi2pee,      NULL, blpi2blpe, },
        { blpe2fei, blpe2fee, blpe2pee, blpe2blpi,      NULL, },
    };

    assert(clockConfig < MAX_MCG_CLOCK_OPTIONS);

    mcgState.nextMode = clockConfigParam[clockConfig].clockMode;

    jumpTable[mcgState.currentMode][mcgState.nextMode](clockConfig);

    /* Store the new clock frequency for clockGetFreq() */
    clockFreq.mcgClockFreq = clockConfigParam[clockConfig].clockHz;
}

/*******************************************************************************
* clockConfigMcgIr
*******************************************************************************/
void clockConfigMcgIr() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigMcgFf
*******************************************************************************/
void clockConfigMcgFf() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigMcgFll
*******************************************************************************/
void clockConfigMcgFll() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigMcgPll
*******************************************************************************/
void clockConfigMcgPll() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigOsc
*******************************************************************************/
void clockConfigOsc() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigOsc32k
*******************************************************************************/
void clockConfigOsc32k() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigEr32k
*******************************************************************************/
void clockConfigEr32k() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigRtc
*******************************************************************************/
void clockConfigRtc() 
{
                                                            /* Not configured */
}

/*******************************************************************************
* clockConfigLpo
*******************************************************************************/
void clockConfigLpo() 
{
                                                            /* Not configured */
}
