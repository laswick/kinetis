/*****************************************************************************
 * flash.c
 *
 * Driver for the FTFL Flash module. In the k60n512 there are two
 * 256kB program flash blocks. Access to either flash block is permitted only
 * if code is not currently executing out of that block. As such any functions
 * that modify flash are executed out of RAM to prevent any possible contention
 *
 ****************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define __RAMCODE__ __attribute__ ((long_call, section(".ramcode")))

static int32_t checkErrors(void)
{
    if ((FTFL_FSTAT & (FTFL_FPVIOL   | FTFL_ACCERR
                    |  FTFL_RDCOLERR | FTFL_MGSTAT0))) {
        return ERROR;
    }
    else {
        return OK;
    }
}

static __RAMCODE__ int32_t safeFlashEraseSector(uint32_t addr)
{
    /* Must be phrase aligned */
    if ((addr & 0x7))
        return ERROR;

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

    return checkErrors();
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

    return OK;
}

int32_t flashInit(const flashConfig_t *cfg)
{
    /* TODO: Clocking. By Default clkdiv is 2
     * and mcg is 50MHz giving max clock
     * of 25MHz */
    /* TODO: Configure FMC for data caching */

    return checkErrors();
}

int32_t flashEraseSector(uint32_t addr)
{
    return safeFlashEraseSector(addr);
}

int32_t flashWrite(uint32_t addr, uint32_t *dataPtr, uint32_t numWords)
{
    return safeFlashWrite(addr, dataPtr, numWords);
}
