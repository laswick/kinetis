/*****************************************************************************
*
* demoMpu.c
*
* Paul Quevedo
*
* This example sets up memory such that a 1K array of data cannot be
* written to by anyone except the debugger. The core has full read access
* to memory but cannot write to the protected region. Once the protected
* region is written to a busFault exception is triggered which will call
* back to the notify function we registered with the MPU.
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
 ****************************************************************************/
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

/******************************************************************************
 * mpuNotify
 *
 *  This function is registered for each region descriptor and called by the
 *  MPU anytime a busFault is caused by an MPU access violation
 *****************************************************************************/
static void mpuNotify(const mpuFaultDesc_t *descPtr)
{
    /* Do something here if you can then return */
    static int faultCount;
    faultCount++;
}

static uint32_t normalArray[8];
static __attribute__ ((aligned (32)))
       uint32_t protectedArray[1024] = { 0x1, 0x2, 0x3, };

#define MPU_ATTR_RWX (MPU_ATTR_READ | MPU_ATTR_WRITE | MPU_ATTR_EXECUTE)
static mpuRegion_t debuggerRegion = {
    .enable = TRUE,
    .startAddr = 0x0,
    .endAddr   = 0xffffffff,
    .attr = {
        [CROSSBAR_MASTER_CODE_BUS] = MPU_ATTR_READ,
        [CROSSBAR_MASTER_SYS_BUS]  = MPU_ATTR_RWX,
    },
    .notifyFn = mpuNotify,
};

static mpuRegion_t coreRegion1 = {
    .enable = TRUE,
    .startAddr = 0x0,
    .endAddr   = (uint32_t)&protectedArray[0] - 1, /* 32bit aligned - 1 */
    .attr = {
        [CROSSBAR_MASTER_CODE_BUS] = MPU_ATTR_RWX,
    },
    .notifyFn = mpuNotify,
};

static mpuRegion_t coreRegion2 = {
    .enable = TRUE,
    .startAddr = (uint32_t)&protectedArray[0] + sizeof(protectedArray),
    .endAddr   = 0xffffffff,
    .attr = {
        [CROSSBAR_MASTER_CODE_BUS] = MPU_ATTR_RWX,
    },
    .notifyFn = mpuNotify,
};


int main(void)
{
    /* The MPU is enabled by default by hardware. We are modifying the regions
     * of the code the core can access. Necessary to disable the MPU while
     * doing this otherwise faults will be detected */
    mpuEnable(FALSE);

    /* Region 0 is allocated by default. Hardware configures it and only the
     * attributes can be modified, debugger exclusive */
    mpuModifyRegion(MPU_REGION0_ID, &debuggerRegion);
    mpuAddRegion(&coreRegion1);
    mpuAddRegion(&coreRegion2);

    mpuEnable(TRUE);

    /* Full access region */
    normalArray[0] = 0x4a4a4a4a;
    /* Protected region allows core read access */
    normalArray[1] = protectedArray[1];
    /* Protected region cannot be written to */
    protectedArray[2] = 0x5a5a5a5a;

    while(1)
        ;
}
