/*******************************************************************************
*
* watchDog.c
*
* Andrew Krajewski
*
* Low level driver for the Kinetis Watchdog module.
* There is mention of watchdogs triggering an ISR however in the assembly
* stage in phase 1 it was discovered that it did not work.
*
* API: watchDogConfig(), watchDogInit(), watchDogKick(), watchDogDisable()
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

#define WDOG_UNLOCK_KEY_1  0xC520
#define WDOG_UNLOCK_KEY_2  0xD928
#define WDOG_REFRESH_KEY_1 0xA602
#define WDOG_REFRESH_KEY_2 0xB480

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
}

/*******************************************************************************
* watchDogInit
*******************************************************************************/
void watchDogInit()
{
}

/*******************************************************************************
* watchDogKick
*******************************************************************************/
void watchDogKick()
{
}

/*******************************************************************************
* watchDogDisable
*******************************************************************************/
void watchDogDisable()
{
}

