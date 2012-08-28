/*******************************************************************************
*
* demoFlash.c
*
* Paul Quevedo
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include "globalDefs.h"
#include "kinetis.h"
#include "hardware.h"

static flashConfig_t flashCfg;

int main(void)
{
    uint32_t buffer[3] = { 0x5a5a5a5a, 0x6a6a6a6a, 0x12345678 };

    flashInit(&flashCfg);
    flashErase(0x20000, FTFL_FLASH_SECTOR_SIZE);
    flashWrite(0x20000, buffer, 3);

    while (1)
        ;

    return 0;
}
