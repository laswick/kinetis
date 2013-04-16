/*******************************************************************************
*
* pit.c
*
* Low level driver for the Kinetis PIT module.
*
* API:
*
* Daryl Hillman
* Sept 2012
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

volatile pitCtrl_t * const pitCtrl
                         = ((volatile pitCtrl_t * const) PIT_CTRL_BASE);

static uint32_t freqToCount(int32_t pitFreq)
{
    int32_t clockHz = clockGetFreq(CLOCK_BUS);
    uint32_t count;

    count = (clockHz / pitFreq) - 1;

    return count;
}


void pitInit(int timer, void *isr, int32_t pitFreqHz)
{
    assert(timer >= 0);
    assert(timer < MAX_PIT);

    SIM_SCGC6 |= SIM_SCGC6_PIT_ENABLE;

    hwInstallISRHandler(ISR_PIT0 + timer, isr);

    pitCtrl->mcr = 1;
    pitCtrl->pit[timer].loadVal = freqToCount(pitFreqHz);
    pitCtrl->pit[timer].ctrl = 0x3;
}


