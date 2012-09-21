/*******************************************************************************
*
* dac.c
*
* Low level driver for the Kinetis DAC module.
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

volatile dac_t * const dac0 = ((volatile dac_t * const) DAC0_BASE_ADDR);

void dac0Init(void)
{
    /*
     * Config the DAC0 Clock Gate
     */

    SIM_SCGC2 |= SIM_SCGC2_DAC0_ENABLE;
}


