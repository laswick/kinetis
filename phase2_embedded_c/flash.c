/*******************************************************************************
*
* flash.c
*
* Paul Quevedo
*
* Driver for the FTFL Flash module. In the k60n512 there are two
* 256kB program flash blocks. Access to either flash block is permitted only
* if code is not currently executing out of that block. As such any functions
* that modify flash are executed out of RAM to prevent any possible contention.
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

static __RAMCODE__ int32_t checkErrors(void)
{
    if ((FTFL_FSTAT & (FTFL_FPVIOL   | FTFL_ACCERR
                    |  FTFL_RDCOLERR | FTFL_MGSTAT0))) {
        return ERROR;
    }
    else {
        return OK;
    }
}

/* Note: Caching of flash data is enabled by default for crossbar masters 0-2
 *       (ARM Instruction/Data buses + DMA). Anytime we modify flash
 *       we must invalidate the cache and prespeculative buffer to ensure
 *       data coherency */
static __RAMCODE__ void flashInvalidateCache(void)
{
    FMC_PFB0CR |= FMC_CINV_WAY;
    FMC_PFB0CR |= FMC_S_B_INV;

    FMC_PFB1CR |= FMC_CINV_WAY;
    FMC_PFB1CR |= FMC_S_B_INV;
}

static __RAMCODE__ int32_t safeFlashErase(uint32_t addr, uint32_t numBytes)
{
    int numSectors;
    int i;

    /* Must be phrase aligned */
    if ((addr & 0x7))
        return ERROR;

    assert((numBytes % FTFL_FLASH_SECTOR_SIZE) == 0);
    numSectors = numBytes / FTFL_FLASH_SECTOR_SIZE;

    for (i = 0; i < numSectors; i++) {
        /* Wait until hardware is idle */
        while (!(FTFL_FSTAT & FTFL_CCIF))
            ;

        FTFL_FCCOB0 = FTFL_CMD_ERASE_FLASH_SECT;
        FTFL_FCCOB1 = (uint8_t)(addr >> 16);
        FTFL_FCCOB2 = (uint8_t)(addr >>  8);
        FTFL_FCCOB3 = (uint8_t)(addr);

        /* Execute command */
        FTFL_FSTAT |= FTFL_CCIF;

        /* Wait until hardware is idle */
        while (!(FTFL_FSTAT & FTFL_CCIF))
            ;

        addr += FTFL_FLASH_SECTOR_SIZE;

        if (checkErrors() == ERROR)
            return ERROR;
    }

    flashInvalidateCache();

    return OK;
}

static __RAMCODE__ int32_t safeFlashWrite(uint32_t addr, uint32_t *dataPtr,
                                                              uint32_t numWords)
{
    uint32_t value;

    if ((addr & 0x7))
        return ERROR;

    /* Wait until hardware is idle */
    while (!(FTFL_FSTAT & FTFL_CCIF))
        ;

    while (numWords--) {
        FTFL_FCCOB0 = FTFL_CMD_PRGRM_LONGWORD;
        FTFL_FCCOB1 = (uint8_t)(addr >> 16);
        FTFL_FCCOB2 = (uint8_t)(addr >>  8);
        FTFL_FCCOB3 = (uint8_t)(addr);
        value = *dataPtr++;
        FTFL_FCCOB4 = (uint8_t)(value >> 24);
        FTFL_FCCOB5 = (uint8_t)(value >> 16);
        FTFL_FCCOB6 = (uint8_t)(value >> 8);
        FTFL_FCCOB7 = (uint8_t)(value);

        /* Execute command */
        FTFL_FSTAT |= FTFL_CCIF;

        /* Wait until hardware is idle */
        while (!(FTFL_FSTAT & FTFL_CCIF))
            ;

        if (checkErrors() == ERROR)
            return ERROR;

        addr += 4;
    }

    flashInvalidateCache();

    return OK;
}

int32_t flashInit(const flashConfig_t *cfg)
{
    /* TODO: Clocking. By Default clkdiv is 2
     * and mcg is 50MHz giving max clock
     * of 25MHz */

    return checkErrors();
}

int32_t flashErase(uint32_t addr, uint32_t numBytes)
{
    return safeFlashErase(addr, numBytes);
}

int32_t flashWrite(uint32_t addr, uint32_t *dataPtr, uint32_t numWords)
{
    return safeFlashWrite(addr, dataPtr, numWords);
}
