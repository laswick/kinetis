/*******************************************************************************
*
* crc.c
*
* Shaun Weise
*
* Low level driver for the Kinetis SPI module.
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

static crcTot_t  fakeTot  = CRC_TOT_NONE;
static crcTotr_t fakeTotr = CRC_TOTR_NONE;
static unsigned (*writeFPtr)(const void *data, unsigned len);

#define SWAP16(x)  ((((x)&0xFF00)>>8)|((x)<<8))
#define SWAP32(x)  (((x) & 0x000000FFU) << 24 | ((x) & 0x0000FF00U) << 8 | \
                    ((x) & 0x00FF0000U) >> 8  | ((x) & 0xFF000000U) >> 24)

/*******************************************************************************/
static uint8_t reverse8(uint8_t byte) {
/*******************************************************************************/
    byte = (byte & 0x0F) << 4 | (byte & 0xF0) >> 4;
    byte = (byte & 0x33) << 2 | (byte & 0xCC) >> 2;
    byte = (byte & 0x55) << 1 | (byte & 0xAA) >> 1;
    return byte;
}

/*******************************************************************************/
static uint16_t reverse16(uint16_t halfword) {
/*******************************************************************************/
    halfword = ((halfword >> 1) & 0x5555) | ((halfword & 0x5555) << 1);
    halfword = ((halfword >> 2) & 0x3333) | ((halfword & 0x3333) << 2);
    halfword = ((halfword >> 4) & 0x0F0F) | ((halfword & 0x0F0F) << 4);
    halfword = ((halfword >> 8) & 0x00FF) | ((halfword & 0x00FF) << 8);
    return halfword;
}

/*******************************************************************************/
static uint32_t reverse32(uint32_t word) {
/*******************************************************************************/
    word = (((word & 0xaaaaaaaa) >> 1) | ((word & 0x55555555) << 1));
    word = (((word & 0xcccccccc) >> 2) | ((word & 0x33333333) << 2));
    word = (((word & 0xf0f0f0f0) >> 4) | ((word & 0x0f0f0f0f) << 4));
    word = (((word & 0xff00ff00) >> 8) | ((word & 0x00ff00ff) << 8));
    return ((word >> 16) | (word << 16));
}

/*******************************************************************************/
static unsigned crcWriteByteWide(const void *data, unsigned len)
/*******************************************************************************/
{
    uint8_t *dataPtr = (uint8_t *) data;
    unsigned i;

    if (CRC_CTRL_TCRC_BB) {                                     /* CRC32 Mode */

        for (i = 0; i < len; i++)
            CRC_DATA_8BIT = *(dataPtr)++;

    } else {                                                    /* CRC16 Mode */

        switch (fakeTot) {                  /* !HACK ALERT! - WOOP WOOP WOOP! */
        default:
        case CRC_TOT_BYTES:
        case CRC_TOT_NONE:
            for (i = 0; i < len; i++)
                CRC_DATA_8BIT = *(dataPtr)++;
        break;
        case CRC_TOT_BITS_AND_BYTES:
        case CRC_TOT_BITS_IN_BYTES:
            for (i = 0; i < len; i++,dataPtr++)
                CRC_DATA_8BIT = reverse8((*(dataPtr)));
        break;
        }
    }
    return len;
}

/*******************************************************************************/
static unsigned crcWrite2ByteWide(const void *data, unsigned len)
/*******************************************************************************/
{
    uint16_t *dataPtr = (uint16_t *) data;
    unsigned i;

    if (CRC_CTRL_TCRC_BB) {                                     /* CRC32 Mode */

        if ( ((unsigned)data % 4) != 0)
            return FALSE;   /* Data ptr must be 32bit align in this CRC32 mode*/
        else if ( (len % 4) != 0 )
            return FALSE;   /* Data must be of multiple 4 bytes */
        for (i = 0; i < len/2; i++) {
                            /* Write 2 16bit values out of order */
            CRC_DATA_16BIT = dataPtr[1-(i%2)];
            if (i%2) dataPtr += 2;
        }

    } else {                                                    /* CRC16 Mode */

        if ( ((unsigned)data % 2) != 0)
            return FALSE;   /* Data ptr must be 16bit align*/
        else if ( (len % 2) != 0 )
            return FALSE;   /* Data must be of multiple 2 bytes */

        switch (fakeTot) {                  /* !HACK ALERT! - WOOP WOOP WOOP! */
        default:
        case CRC_TOT_NONE:
            for (i = 0; i < len/2; i++, dataPtr++)
                CRC_DATA_16BIT = *(dataPtr);
        break;
        case CRC_TOT_BITS_IN_BYTES:
            for (i = 0; i < len/2; i++,dataPtr++) {
                CRC_DATA_16BIT = ( (reverse8(((*(dataPtr))&0xFF00)>>8)<<8) \
                                             | reverse8(((*(dataPtr))&0xFF)) );
            }
        break;
        case CRC_TOT_BITS_AND_BYTES:
            for (i = 0; i < len/2; i++, dataPtr++)
                CRC_DATA_16BIT = reverse16(*(dataPtr));
        break;
        case CRC_TOT_BYTES:
            for (i = 0; i < len/2; i++, dataPtr++)
                CRC_DATA_16BIT = SWAP16(*(dataPtr));
        break;
        }
    }
    return len;
}

/*******************************************************************************/
static unsigned crcWrite4ByteWide(const void *data, unsigned len)
/*******************************************************************************/
{
    unsigned i;
    uint32_t *dataPtr = (uint32_t *)data;

    /* Checks */
    if (!CRC_CTRL_TCRC_BB)
        return FALSE;   /* CRC mode must be CRC32 */
    else if ( ((unsigned)data % 4) != 0)
        return FALSE;   /* Data ptr must be 32bit align*/
    else if ( (len % 4) != 0 )
        return FALSE;   /* Data must be of multiple 4 bytes */

    for (i= 0; i < len/4 ; i++, dataPtr++)
        CRC_DATA = *(dataPtr);

    return len;
}

/*******************************************************************************
 * POSIX FUNCTIONS
 *******************************************************************************/

/*******************************************************************************/
static int crc_ioctl(devoptab_t *dot, int cmd,  int flags)
/*******************************************************************************/
{
    unsigned reg;

//    if (!dot || !dot->priv) return FALSE;
    if (cmd < 0 || cmd > MAX_IO_IOCTRL_CRC_CMDS) return FALSE;

    switch (cmd) {
    case IO_IOCTL_CRC_SET_DWW:
        if (flags >= 0 && flags < MAX_CRC_DWW) {
            switch (flags) {
            default:
            case CRC_DWW_BYTE:
                writeFPtr = crcWriteByteWide;
            break;
            case CRC_DWW_2BYTE:
                writeFPtr = crcWrite2ByteWide;
            break;
            case CRC_DWW_4BYTE:
                writeFPtr = crcWrite4ByteWide;
            break;
            }
        } else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_SET_TOT:
        if (flags >= 0 && flags < MAX_CRC_TOT) {
            if (CRC_CTRL_TCRC_BB) {
                reg = CRC_CTRL & ~(CRC_CTRL_TOT_MSK);
                CRC_CTRL = reg | (flags << CRC_CTRL_TOT_BIT);
                fakeTot = CRC_TOT_NONE;
            } else {
                fakeTot = flags;
            }
        } else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_SET_TOTR:
        if (flags >= 0 && flags < MAX_CRC_TOTR) {
            if (CRC_CTRL_TCRC_BB) {
                reg = CRC_CTRL & ~(CRC_CTRL_TOTR_MSK);
                CRC_CTRL = reg | (flags << CRC_CTRL_TOTR_BIT);
                fakeTotr = CRC_TOTR_NONE;
            } else {
                fakeTotr = flags;
            }
        } else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_SET_FXOR:
        if (flags >= 0 && flags < MAX_CRC_FXOR) {
            if (flags == CRC_FXOR_DISABLE)
                CRC_CTRL_FXOR_BB = 0;
            else
                CRC_CTRL_FXOR_BB = 1;
        } else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_SET_SEED:
        CRC_CTRL_WAS_BB = 1;
        if (CRC_CTRL_TCRC_BB) {               /* CRC32 Mode */
            CRC_DATA = (uint32_t) flags;
        } else {                              /* CRC16 Mode */
            CRC_DATA_16BIT = (uint16_t) flags;
        }
        CRC_CTRL_WAS_BB = 0;
    break;
    case IO_IOCTL_CRC_SET_POLY:
         if (CRC_CTRL_TCRC_BB) {              /* CRC32 Mode */
            CRC_GPOLY = (uint32_t) flags;
         } else {                              /* CRC16 Mdoe */
            CRC_GPOLY_16BIT = (uint16_t) flags;
         }
    break;
    case IO_IOCTL_CRC_SET_TYPE:
        if (flags >= 0 && flags < MAX_CRC_TYPES) {
            if (flags == CRC_TYPE_16) {
                CRC_CTRL_TCRC_BB = 0;
                fakeTot =((CRC_CTRL & CRC_CTRL_TOT_MSK ) >> CRC_CTRL_TOT_BIT );
                fakeTotr=((CRC_CTRL & CRC_CTRL_TOTR_MSK) >> CRC_CTRL_TOTR_BIT);
                CRC_CTRL_TOT1_BB = 0;
                CRC_CTRL_TOT0_BB = 0;
                CRC_CTRL_TOTR1_BB = 0;
                CRC_CTRL_TOTR0_BB = 0;
            } else {
                CRC_CTRL_TCRC_BB = 1;
                fakeTot = CRC_TOT_NONE;
                fakeTotr = CRC_TOTR_NONE;
            }
        } else {
            assert(0);
            return FALSE;
        }
    break;
    default:
        assert(0);
        return FALSE;
    break;
    }

    return TRUE;
}

/*******************************************************************************/
static long crc_write_r (void *reent, devoptab_t *dot, const void *buf, int len )
/*******************************************************************************/
{
    //if (!dot || !dot->priv) return FALSE;
    return writeFPtr(buf, len);
}

/*******************************************************************************/
static long crc_read_r (void *reent, devoptab_t *dot, void *buf, int len )
/*******************************************************************************/
{
    union {
        uint32_t *u32;
        uint16_t *u16;
    } dataPtr;

    if (CRC_CTRL_TCRC_BB && len == 4) {               /* 32 Bit Mode */
        dataPtr.u32 = (uint32_t *)buf;
        *(dataPtr.u32) = CRC_DATA;
        return 4;

    } else if (len == 2) {                            /* 16 Bit Mode */
        dataPtr.u16 = (uint16_t *)buf;

        switch (fakeTotr) {
        default:
        case CRC_TOTR_NONE:
            /* Read LU, LL */
            *(dataPtr.u32) = (uint32_t)CRC_DATA;
        break;
        case CRC_TOTR_BITS_IN_BYTES:
            /* Read LU, LL */
            *(dataPtr.u16) = ( (reverse8((CRC_DATA&0xFF00)>>8)<<8) \
                                         | reverse8((CRC_DATA&0xFF)) )>>16;
        break;
        case CRC_TOTR_BITS_AND_BYTES:
            /* Read HU, HL After a Btye Transposal */
            *(dataPtr.u16) = reverse32(CRC_DATA)>>16;
        break;
        case CRC_TOTR_BYTES:
            /* Read HU, HL After a Byte Transposal */
            *(dataPtr.u16) = (SWAP32(CRC_DATA))>>16;
        break;
        }
        return 2;
    }
    return 0;
}

/*******************************************************************************/
static int crc_close_r (void *reent, devoptab_t *dot )
/*******************************************************************************/
{
    if (!dot || !dot->priv) return FALSE;

    SIM_SCGC6_CRC_BB = 0; /* Disable the SIMSCGC for the crc module being used*/
    return TRUE;
}

/*******************************************************************************/
static int crc_open_r (void *reent, devoptab_t *dot, int mode, int flags )
/*******************************************************************************/
{
    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }
    if (strcmp(DEVOPTAB_CRC_STR, dot->name) != 0) {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }
    /* Check if device is open */
    if (dot->priv) {
        ((struct _reent *)reent)->_errno = EPERM;
        return FALSE;
    }

    SIM_SCGC6_CRC_BB = 1;   /* Enable the CRC in the SIM */

    return TRUE;
}

/******************************************************************************/
int crc_install(void)
/******************************************************************************/
{
    if (deviceInstall(DEV_MAJ_CRC, crc_open_r, crc_ioctl, crc_close_r,
                                                     crc_write_r, crc_read_r)) {
        if (deviceRegister(DEVOPTAB_CRC_STR, DEV_MAJ_CRC, 0, NULL))
            return TRUE;
    }
    return FALSE;
}
