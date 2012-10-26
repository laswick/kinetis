/*******************************************************************************
*
* sdhc.c
*
* Geoff Chapman 
*
* Low level driver for the Kinetis SDHC module.
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"


inline uint32_t swap(uint32_t x)
{
    x = (x>>24) | 
        ((x<<8) & 0x00FF0000) |
        ((x>>8) & 0x0000FF00) |
        (x<<24);

    return x;
}


static inline void setBits32(volatile uint32_t *addr, uint32_t value,
                                                  uint32_t shift, uint32_t mask)
{
    (*addr) = (*addr & ~mask) | ((value << shift) & mask);
}

static inline uint32_t getBits32(volatile uint32_t *addr,
                                                  uint32_t shift, uint32_t mask)
{
    return ((*addr & mask) >> shift);
}

static inline void setBits8(volatile uint8_t *addr, uint8_t value,
                                                    uint8_t shift, uint8_t mask)
{
    (*addr) = (*addr & ~mask) | ((value << shift) & mask);
}

static inline uint8_t getBits8(volatile uint8_t *addr,
						    uint8_t shift, uint8_t mask)
{
    return ((*addr & mask) >> shift);
}

/* 
 * For CIC (Cmd Indx Check) and CCC (Cmd CRC Check) xferType flags, 
 * see table 52-8 in Freescale Kinetis Technical Referance Manual (TRM).
 */
sdhcCmd_t sdhcCmd00 = {
    .cmdIndx  = 0,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_NONE,
    .xferType = 0, 
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd02 = {
    .cmdIndx  = 2,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_136,
    .xferType = SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd03 = {
    .cmdIndx  = 3,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd07 = {
    .cmdIndx  = 7,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd08 = {
    .cmdIndx  = 8,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0x000001AA,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd09 = {
    .cmdIndx  = 9,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_136,
    .xferType = SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd13 = {
    .cmdIndx  = 13,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd17 = {
    .cmdIndx  = 17,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK
              | SDHC_XFERTYP_DPSEL_MASK | SDHC_XFERTYP_DTDSEL_MASK,
    .arg      = 1,
    .read     = 1,
    .nBlks    = 1,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd24 = {
    .cmdIndx  = 24,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK
              | SDHC_XFERTYP_DPSEL_MASK,
    .arg      = 1,
    .read     = 0,
    .nBlks    = 1,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd32 = {
    .cmdIndx  = 32,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd33 = {
    .cmdIndx  = 33,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd38 = {
    .cmdIndx  = 38,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcCmd55 = {
    .cmdIndx  = 55,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = SDHC_XFERTYP_CICEN_MASK | SDHC_XFERTYP_CCCEN_MASK,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

sdhcCmd_t sdhcACmd41= {
    .cmdIndx  = 41,
    .cmdType  = SDHC_CMD_TYPE_NORMAL,
    .rspType  = SDHC_RSP_TYPE_LEN_48,
    .xferType = 0,
    .arg      = 0,
    .read     = 0,
    .nBlks    = 0,
    .resp     = { 0 },
};

/*******************************************************************************
 * sdhcSetPCR 
 * 
 *
 ******************************************************************************/
void sdhcSetPCR(void)
{
    /*
     * Configure Port E Control Register for SDHC functionality (ALT4) 
     */
    /* D1  */
    PORT_PCR(PORTE, 0) |= (PORT_MUX_ALT4 | PORT_DSE | PORT_PULLUP_ENABLE);
    /* D0  */
    PORT_PCR(PORTE, 1) |= (PORT_MUX_ALT4 | PORT_DSE | PORT_PULLUP_ENABLE);
    /* CLK */
    PORT_PCR(PORTE, 2) |= (PORT_MUX_ALT4 | PORT_DSE );
    /* CMD */
    PORT_PCR(PORTE, 3) |= (PORT_MUX_ALT4 | PORT_DSE | PORT_PULLUP_ENABLE);
    /* D3  */
    PORT_PCR(PORTE, 4) |= (PORT_MUX_ALT4 | PORT_DSE );
#if 0
    PORT_PCR(PORTE, 4) |= (PORT_MUX_ALT4 | PORT_DSE | PORT_PULLUP_ENABLE);
#endif
    /* D2  */
    PORT_PCR(PORTE, 5) |= (PORT_MUX_ALT4 | PORT_DSE | PORT_PULLUP_ENABLE);
}

/*******************************************************************************
 *
 * sdhcInit
 *
 ******************************************************************************/
int sdhcInit(void)
{
    /*
     * Set bit in System Clock Gating Congrol Register 5 (SCGC5) in the SIM
     * to enable the PORTE clock gate
     */
    SIM_SCGC5 |= SIM_SCGC5_PORTE_ENABLE;

    SIM_SCGC3 |= SIM_SCGC3_SDHC_ENABLE;

    /*
     * Set the Port Control Registers (PCR) for SDHC functionality
     */
    sdhcSetPCR();

    setBits32(&SDHC_SYSCTL_REG, 0x02, SDHC_SYSCTL_SDCLKFS_SHIFT,
                                      SDHC_SYSCTL_SDCLKFS_MASK );

    /*
     * Reset the SDHC module. See section 52.6.2.2 of TRM
     */
    setBits32(&SDHC_SYSCTL_REG, 0x01, SDHC_SYSCTL_RSTA_SHIFT,
                                      SDHC_SYSCTL_RSTA_MASK);

    while (getBits32(&SDHC_SYSCTL_REG, SDHC_SYSCTL_RSTA_SHIFT,
                                       SDHC_SYSCTL_RSTA_MASK ))
    { }

    /* 
     * Before sending the required 80 clocks to the SDHC card, make sure
     * command/data lines are not active.  See section 52.7.1 of TRM.
     */
    while (getBits32(&SDHC_PRSSTAT_REG, SDHC_PRSSTAT_CIHB_SHIFT,
                                        SDHC_PRSSTAT_CIHB_MASK )
        || getBits32(&SDHC_PRSSTAT_REG, SDHC_PRSSTAT_CDIHB_SHIFT,
                                        SDHC_PRSSTAT_CDIHB_MASK))
    { }

    /*
     * Send 80 clocks to the SDHC card
     */
    setBits32(&SDHC_SYSCTL_REG, 1, SDHC_SYSCTL_INITA_SHIFT,
                                   SDHC_SYSCTL_INITA_MASK );

    while (getBits32(&SDHC_SYSCTL_REG, SDHC_SYSCTL_INITA_SHIFT,
                                       SDHC_SYSCTL_INITA_MASK ))
    { }

    /* 
     * Since we are doing single block read/writes set block count to 1
     * for now
     */
    setBits32(&SDHC_BLKATTR_REG, 0x01, SDHC_BLKATTR_BLKCNT_SHIFT,
                                       SDHC_BLKATTR_BLKCNT_MASK );

    /*
     * Block size is always 512 bytes for SDHC cards
     */
    setBits32(&SDHC_BLKATTR_REG,  512, SDHC_BLKATTR_BLKSIZE_SHIFT,
                                       SDHC_BLKATTR_BLKSIZE_MASK );

    /*
     * Set read/write watermark levels to 1 word
     */
    setBits32(&SDHC_WML_REG,     0x01, SDHC_WML_RDWML_SHIFT,
                                       SDHC_WML_RDWML_MASK );

    setBits32(&SDHC_WML_REG,     0x01, SDHC_WML_WRWML_SHIFT,
                                       SDHC_WML_WRWML_MASK );

    SDHC_IRQSTAT_REG   = 0xFFFFFFFF;

    return 0;
}


/*******************************************************************************
 * sdhcSendCmd
 * 
 * This routine implements the steps outlined in Section 52.6.1 of the Kinetis
 * k60 Technical Reference Manual
 * (K60P144M100SF2RM.pdf)
 *
 ******************************************************************************/
int sdhcSendCmd(sdhcCmd_t *command)
{
    uint32_t xfertype;
    uint32_t waitCount;
    
    /*
     * Set CICEN, CCCEN, RSPTYP according to the command index (Section 52.6)
     */

    xfertype = command->xferType;

    /*
     * Set cmdTyp according to the command index (TRM Section 52.6)
     */
    setBits32(&xfertype, command->cmdType, SDHC_XFERTYP_CMDTYP_SHIFT, 
                                           SDHC_XFERTYP_CMDTYP_MASK);

    setBits32(&xfertype, command->cmdIndx, SDHC_XFERTYP_CMDINX_SHIFT, 
                                           SDHC_XFERTYP_CMDINX_MASK);

    setBits32(&xfertype, command->rspType, SDHC_XFERTYP_RSPTYP_SHIFT, 
                                           SDHC_XFERTYP_RSPTYP_MASK);

    /*
     * Write the command argument  (TRM Section 52.6.1)
     */
    SDHC_CMDARG_REG = command->arg;

    /*
     * Issue command by writing to XFERTYP register (TRM Section 52.6.1)
     */
    SDHC_XFERTYP_REG = xfertype;
    
    /*
     * Wait until Command Complete (CC) bit is set (TRM Section 52.6.1)
     */
    do {
        waitCount++;
    }
    while ((SDHC_IRQSTAT_REG & SDHC_IRQSTAT_CC_MASK) != SDHC_IRQSTAT_CC_MASK);

    /*
     * Grab response only if particular command generates one
     */
    if (command->rspType != SDHC_RSP_TYPE_NONE)
    {
        command->resp[0] = SDHC_CMDRSP_REG_0;

        if (command->rspType == SDHC_RSP_TYPE_LEN_136)
        {
            command->resp[1] = SDHC_CMDRSP_REG_1;
            command->resp[2] = SDHC_CMDRSP_REG_2;
            command->resp[3] = SDHC_CMDRSP_REG_3;
        }
    }

    /*
     * Write 1 to clear CC bit (TRM Section 52.6.1)
     */
    SDHC_IRQSTAT_REG |= SDHC_IRQSTAT_CC_MASK;

    return 0;
}

/*******************************************************************************
 * sdhcRead
 * 
 *
 ******************************************************************************/
int sdhcRead (uint32_t *ptr, int n)
{
    uint32_t i = n;
    uint32_t j;

    for (j = (i + 3) / 4; j != 0; j--)
    {
        while (!getBits32(&SDHC_PRSSTAT_REG, SDHC_PRSSTAT_BREN_SHIFT,
                                             SDHC_PRSSTAT_BREN_MASK))
        { }
    
        *ptr++ = SDHC_DATPORT_REG;
    }

    return n;
}

/*******************************************************************************
 * sdhcWrite
 * 
 *
 ******************************************************************************/
int sdhcWrite (const uint32_t *ptr, int n)
{
    uint32_t i = n;
    uint32_t j;

    for (j = (i + 3) / 4; j != 0; j--)
    {
        while (!getBits32(&SDHC_PRSSTAT_REG, SDHC_PRSSTAT_BWEN_SHIFT,
                                             SDHC_PRSSTAT_BWEN_MASK))
        { }
    
        SDHC_DATPORT_REG = *ptr++;
    }

    return n;
}

/*******************************************************************************
 * sdhc_open_r
 *
 ******************************************************************************/
static int sdhc_open_r (void *reent, devoptab_t *dot, int mode, int flags )
{
    uint32_t address;

    sdhcInit();

    sdhcSendCmd(&sdhcCmd00);
    sdhcSendCmd(&sdhcCmd08);

    sdhcSendCmd(&sdhcCmd55);
    sdhcSendCmd(&sdhcACmd41);

    sdhcACmd41.arg = 0x40300000;

    do {
        sdhcSendCmd(&sdhcCmd55);
        sdhcSendCmd(&sdhcACmd41);
    } while ((sdhcACmd41.resp[0] & 0x80000000) == 0);

    sdhcSendCmd(&sdhcCmd02);
    sdhcSendCmd(&sdhcCmd03);

    address = sdhcCmd03.resp[0] & 0xFFFF0000;

    sdhcCmd09.arg = address;
    sdhcSendCmd(&sdhcCmd09);

    sdhcCmd07.arg = address;
    sdhcSendCmd(&sdhcCmd07);

    sdhcCmd13.arg = address;
    sdhcSendCmd(&sdhcCmd13);

    return TRUE;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int sdhc_close_r (void *reent, devoptab_t *dot)
{
    return 1;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int sdhc_ioctl (devoptab_t *dot, int cmd, int flags)
{

    switch (cmd) {
    case IO_IOCTL_SDHC_SET_READ_BLOCK:
        sdhcCmd17.arg = flags;
        break;
    case IO_IOCTL_SDHC_SET_WRITE_BLOCK:
        sdhcCmd24.arg = flags;
        break;
    case IO_IOCTL_SDHC_ERASE_BLOCK:
        sdhcCmd32.arg = flags;
        sdhcCmd33.arg = flags;
        sdhcSendCmd(&sdhcCmd32);
        sdhcSendCmd(&sdhcCmd33);
        sdhcSendCmd(&sdhcCmd38);
        break;
    case IO_IOCTL_SDHC_GET_CID:
        {
            cidReg_t *cidPtr = (cidReg_t *) flags;
            cidPtr->raw[0] = swap(sdhcCmd02.resp[3]);
            cidPtr->raw[1] = swap(sdhcCmd02.resp[2]);
            cidPtr->raw[2] = swap(sdhcCmd02.resp[1]);
            cidPtr->raw[3] = swap(sdhcCmd02.resp[0]);
        }
        break;
    }

    return 1;
}

/*******************************************************************************
 *
 ******************************************************************************/
static long sdhc_write_r (void *reent, devoptab_t *dot, const void *buffer,
                                                                        int len)
{
    sdhcSendCmd(&sdhcCmd24);
    sdhcWrite(buffer, 512);
    
    return len;
}

/******************************************************************************
 *
 ******************************************************************************/
static long sdhc_read_r (void *reent, devoptab_t *dot, void *buffer, int len)
{
    sdhcSendCmd(&sdhcCmd17);
    sdhcRead(buffer, 512);

    return len;
}

/******************************************************************************/
int sdhc_install(void)
/******************************************************************************/
{
    int ret = TRUE;

    if( !deviceInstall(DEV_MAJ_SDHC, sdhc_open_r, sdhc_ioctl, sdhc_close_r,
                                                   sdhc_write_r, sdhc_read_r)) {
        ret = FALSE;
    }
    if( !deviceRegister("sdhc0", DEV_MAJ_SDHC, 0,  NULL) ) {
        ret =  FALSE;
    }

    return ret;
}
