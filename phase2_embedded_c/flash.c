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

enum {
    SWAP_STATE_UNINITIALIZED,
    SWAP_STATE_READY,
    SWAP_STATE_UPDATE,
    SWAP_STATE_UPDATE_ERASED,
    SWAP_STATE_COMPLETE,
};

enum {
    SWAP_CMD_INITIALIZE = 0x01,
    SWAP_CMD_UPDATE     = 0x02,
    SWAP_CMD_COMPLETE   = 0x04,
    SWAP_CMD_STATUS     = 0x08,
};

extern uint32_t _flash_swap_addr;

static inline __RAMCODE__ void executeFlashCmd(void)
{
    /* Execute command */
    FTFL_FSTAT |= FTFL_CCIF;

    /* Wait until hardware is idle */
    while (!(FTFL_FSTAT & FTFL_CCIF))
        ;
}

static int32_t checkErrors(void)
{
    uint32_t error = (FTFL_FSTAT & (FTFL_FPVIOL   | FTFL_ACCERR
                   |  FTFL_RDCOLERR | FTFL_MGSTAT0));
    if (error) {
        FTFL_FSTAT |= (FTFL_FPVIOL | FTFL_ACCERR | FTFL_RDCOLERR);
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
static inline void flashInvalidateCache(void)
{
    FMC_PFB0CR |= FMC_CINV_WAY;
    FMC_PFB0CR |= FMC_S_B_INV;

    FMC_PFB1CR |= FMC_CINV_WAY;
    FMC_PFB1CR |= FMC_S_B_INV;
}

int32_t flashInit(const flashConfig_t *cfg)
{
    /* TODO: Clocking. By Default clkdiv is 2
     * and mcg is 50MHz giving max clock
     * of 25MHz */

    return checkErrors();
}

static int32_t flashEraseCmd(uint32_t addr, uint32_t numBytes)
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

        executeFlashCmd();

        addr += FTFL_FLASH_SECTOR_SIZE;

        if (checkErrors() == ERROR)
            return ERROR;
    }

    flashInvalidateCache();

    return OK;
}
int32_t flashErase(uint32_t addr, uint32_t numBytes)
{
    return flashEraseCmd(addr, numBytes);
}

int32_t flashEraseBlock(uint32_t blockNum)
{
    uint32_t addr = blockNum * FTFL_FLASH_BLOCK_SIZE;

    if (blockNum > MAX_FLASH_BLOCKS)
        return ERROR;

    while(!(FTFL_FSTAT & FTFL_CCIF))
        ;

    FTFL_FCCOB0 = FTFL_CMD_ERASE_FLASH_BLOCK;
    FTFL_FCCOB1 = (uint8_t)(addr >> 16);
    FTFL_FCCOB2 = (uint8_t)(addr >>  8);
    FTFL_FCCOB3 = (uint8_t)(addr);

    executeFlashCmd();

    if (checkErrors() == ERROR)
        return ERROR;

    flashInvalidateCache();

    return OK;
}

int32_t flashWrite(uint32_t addr, uint32_t *dataPtr, uint32_t numWords)
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

        executeFlashCmd();

        if (checkErrors() == ERROR)
            return ERROR;

        addr += 4;
    }

    flashInvalidateCache();

    return OK;
}

static int32_t flashSwapCmd(uint32_t swapCmd)
{
    int32_t swapState;
    uint32_t flash_swap_addr = (uint32_t)&_flash_swap_addr;

    if (flash_swap_addr == 0xffffffff)
        return ERROR;

    while(!(FTFL_FSTAT & FTFL_CCIF))
        ;

    FTFL_FCCOB0 = FTFL_CMD_SWAP;
    FTFL_FCCOB1 = (uint8_t)(flash_swap_addr >> 16);
    FTFL_FCCOB2 = (uint8_t)(flash_swap_addr >>  8);
    FTFL_FCCOB3 = (uint8_t)(flash_swap_addr);
    FTFL_FCCOB4 = swapCmd;

    executeFlashCmd();

    if (checkErrors() == ERROR)
        swapState = ERROR;
    else
        swapState = FTFL_FCCOB5;

    return swapState;
}

int32_t flashSwapInit(void)
{
    int swapState = flashSwapCmd(SWAP_CMD_STATUS);

    switch (swapState) {
    default:
    case ERROR:
        return ERROR;
    case SWAP_STATE_UNINITIALIZED:
        if (flashSwapCmd(SWAP_CMD_INITIALIZE) == ERROR)
            return ERROR;
        if (flashSwapCmd(SWAP_CMD_STATUS) != SWAP_STATE_UPDATE_ERASED)
            return ERROR;
        break;
    case SWAP_STATE_READY:
    case SWAP_STATE_COMPLETE:
        if (flashSwapCmd(SWAP_CMD_UPDATE) == ERROR)
            return ERROR;
        if (flashSwapCmd(SWAP_CMD_STATUS) != SWAP_STATE_UPDATE)
            return ERROR;
        break;
    case SWAP_STATE_UPDATE:
    case SWAP_STATE_UPDATE_ERASED:
        break;
    }
    return OK;
}

int32_t flashSwap(void)
{
    if (flashSwapCmd(SWAP_CMD_STATUS) != SWAP_STATE_UPDATE_ERASED)
        return ERROR;
    if (flashSwapCmd(SWAP_CMD_COMPLETE) == ERROR)
        return ERROR;
    if (flashSwapCmd(SWAP_CMD_STATUS) != SWAP_STATE_COMPLETE)
        return ERROR;

    return OK;
}
