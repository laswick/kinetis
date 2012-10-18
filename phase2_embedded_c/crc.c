/*******************************************************************************
*
* crc.c
*
* Shaun Weise
*
* Low level driver for the Kinetis SPI module.
*
* TODO: Bit banding
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

typedef struct crc_s {
    volatile uint32_t * crc;
    volatile uint32_t * crc32;
    volatile uint16_t * crc16;
    volatile uint8_t  * crc8;
    volatile uint32_t * gpoly32;
    volatile uint16_t * gpoly16;
    volatile uint32_t * ctrl;
    volatile uint32_t * simScgc;
    uint32_t            seed;
    unsigned            dww;
    unsigned            (*write)(struct crc_s *crc, const void *data, unsigned len);
    unsigned            simScgcEnBit;
} crc_t;

static unsigned crcWriteByteWide (crc_t *crc, const void *data, unsigned len);
static unsigned crcWrite2ByteWide(crc_t *crc, const void *data, unsigned len);
static unsigned crcWrite4ByteWide(crc_t *crc, const void *data, unsigned len);

crc_t crcDflt = {
    .crc          = CRC_CRC_PTR,
    .crc32        = CRC_CRC32_PTR,
    .crc16        = CRC_CRC16_PTR,
    .crc8         = CRC_CRC8_PTR,
    .gpoly32      = CRC_GPOLY32_PTR,
    .gpoly16      = CRC_GPOLY16_PTR,
    .ctrl         = CRC_CTRL_PTR,
    .simScgc      = SIM_SCGC6_PTR,
    .seed         = 0xA535A535,
    .dww          = CRC_DWW_BYTE,
    .write        = crcWriteByteWide,
    .simScgcEnBit = SIM_SCGC6_CRC_ENABLE,
};

/*******************************************************************************/
static int crcOpen(devoptab_t *dot)
/*******************************************************************************/
{
    crc_t *crc;

    if (dot->priv) return FALSE; /* Device is already open */

    /* Create 'private' crc structure and point devoptab's
     * private pointer to it */
    crc = (crc_t *) malloc(sizeof(crc_t));
    if (!crc) return FALSE;
    else dot->priv = crc;

    /* Load init & default info into private crc structure */
    memcpy(crc, &crcDflt, sizeof(crc_t));

    /* Enable the SIMSCGC for the crc module being used*/
    *crc->simScgc |= crc->simScgcEnBit;

    return TRUE;
}

/*******************************************************************************/
static unsigned crcWriteByteWide(crc_t *crc, const void *data, unsigned len)
/*******************************************************************************/
{
    volatile uint8_t *crcPtr;
    uint8_t *dataPtr;
    unsigned i;

    /* Write */
    dataPtr = (uint8_t *) data;
    crcPtr = (uint8_t *) crc->crc;
    if ( *crc->ctrl & CRC_CTRL_TCRC ) {  /* CRC32 Mode */
        for (i = 0; i < len; i++)
            crcPtr[i%4] = *(dataPtr)++;
    } else {                             /* CRC16 Mode */
        for (i = 0; i < len; i++)
            crcPtr[i%2] = *(dataPtr)++;
    }

    return len;
}

/*******************************************************************************/
static unsigned crcWrite2ByteWide(crc_t *crc, const void *data, unsigned len)
/*******************************************************************************/
{
    volatile uint16_t *crcPtr;
    uint16_t *dataPtr;
    unsigned i;

    /* Checks */
    if ( ((unsigned)data % 2) != 0)
        return FALSE;   /* Data ptr must be 16bit align*/
    else if ( (len % 2) != 0 )
        return FALSE;   /* Data must be of multiple 2 bytes */

    /* Write */
    dataPtr = (uint16_t *) data;
    crcPtr = (uint16_t *) crc->crc;
    if ( *crc->ctrl & CRC_CTRL_TCRC ) {  /* CRC32 Mode */
        for (i = 0; i < len/2; i++)
            crcPtr[i%2] = *(dataPtr)++;
    } else {                             /* CRC16 Mode */
        for (i = 0; i < len/2; i++)
            *(crcPtr) = *(dataPtr)++;
    }

    return i;
}

/*******************************************************************************/
static unsigned crcWrite4ByteWide(crc_t *crc, const void *data, unsigned len)
/*******************************************************************************/
{
    unsigned i;
    uint32_t *dataPtr;

    /* Checks */
    if ( !(*crc->ctrl & CRC_CTRL_TCRC) )
        return FALSE;   /* CRC mode must be CRC32 */
    else if ( ((unsigned)data % 4) != 0)
        return FALSE;   /* Data ptr must be 32bit align*/
    else if ( (len % 4) != 0 )
        return FALSE;   /* Data must be of multiple 4 bytes */

    /* Write */
    dataPtr = (uint32_t *) data;
    for (i = 0; i < len/4; i++)
        *(crc->crc) = *(dataPtr)++;
    return i;
}

/*******************************************************************************
 * POSIX FUNCTIONS
 *******************************************************************************/

/*******************************************************************************/
static int crc_open_r (void *reent, devoptab_t *dot, int mode, int flags )
/*******************************************************************************/
{
    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }

    /* Check the device name */
    if (strcmp(DEVOPTAB_CRC_STR, dot->name) != 0) {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }
    else {
        /* Try to open if not already open */
        if (crcOpen(dot)) {
            return TRUE;
        }
        else {
            /* Device is already open */
            ((struct _reent *)reent)->_errno = EPERM;
            return FALSE;
        }
    }
}

/*******************************************************************************/
static int crc_ioctl(devoptab_t *dot, int cmd,  int flags)
/*******************************************************************************/
{
    crc_t *crc;
    unsigned reg;

    if (!dot || !dot->priv) return FALSE;
    else crc = (crc_t *) dot->priv;

    if (cmd < 0 || cmd > MAX_IO_IOCTRL_CRC_CMDS) return FALSE;

    switch (cmd) {
    case IO_IOCTL_CRC_SET_DWW:
        if (flags >= 0 && flags < MAX_CRC_DWW) {
            crc->dww = flags;
            switch (flags) {
            default:
            case CRC_DWW_BYTE:
                crc->write = crcWriteByteWide;
            break;
            case CRC_DWW_2BYTE:
                crc->write = crcWrite2ByteWide;
            break;
            case CRC_DWW_4BYTE:
                crc->write = crcWrite4ByteWide;
            break;
            }
        }
        else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_GET_DWW:
        return (crc->dww);
    break;
    case IO_IOCTL_CRC_SET_TOT:
        if (flags >= 0 && flags < MAX_CRC_TOT) {
            reg = *crc->ctrl & ~(CRC_CTRL_TOT);
            *crc->ctrl = reg | (flags << 30);
        }
        else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_GET_TOT:
        return ((*crc->ctrl >> 30) & 0x3);
    break;
    case IO_IOCTL_CRC_SET_TOTR:
        if (flags >= 0 && flags < MAX_CRC_TOTR) {
            reg = *crc->ctrl & ~(CRC_CTRL_TOTR);
            *crc->ctrl = reg | (flags << 28);
        }
        else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_GET_TOTR:
        return ((*crc->ctrl >> 28) & 0x3);
    break;
    case IO_IOCTL_CRC_SET_FXOR:
        if (flags >= 0 && flags < MAX_CRC_FXOR) {
            if (flags == CRC_FXOR_DISABLE)
                *crc->ctrl &= ~(CRC_CTRL_FXOR);
            else
                *crc->ctrl |= CRC_CTRL_FXOR;
        }
        else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_GET_FXOR:
        return ((*crc->ctrl >> 26) & 0x1);
    break;
    case IO_IOCTL_CRC_SET_SEED:
        *crc->ctrl |= CRC_CTRL_WAS;
        //CRC_BB_CTRL_WAS = 1;
        if ( *crc->ctrl & CRC_CTRL_TCRC ) {  /* CRC32 Mode */
            *crc->crc32 = (uint32_t) flags;
        } else {                             /* CRC16 Mode */
            *crc->crc16 = (uint16_t) flags;
        }
        //CRC_BB_CTRL_WAS = 0;
        *crc->ctrl &= ~(CRC_CTRL_WAS);
        crc->seed = flags;
    break;
    case IO_IOCTL_CRC_GET_SEED:
        return crc->seed;
    break;
    case IO_IOCTL_CRC_SET_POLY:
         if ( *crc->ctrl & CRC_CTRL_TCRC ) {  /* CRC32 Mode */
            *crc->gpoly32 = (uint32_t) flags;
        } else {                              /* CRC16 Mdoe */
            *crc->gpoly16 = (uint16_t) flags;
        }
    break;
    case IO_IOCTL_CRC_GET_POLY:
        return (int)*crc->gpoly32;
    break;
    case IO_IOCTL_CRC_SET_PRO_WIDTH:
        if (flags >= 0 && flags < MAX_CRC_WIDTH) {
            if (flags == CRC_WIDTH_16)
                *crc->ctrl &= ~(CRC_CTRL_TCRC);
            else
                *crc->ctrl |= CRC_CTRL_TCRC;
        }
        else {
            assert(0);
            return FALSE;
        }
    break;
    case IO_IOCTL_CRC_GET_PRO_WIDTH:
        return ((*crc->ctrl >> 24) & 0x1);
    break;
    default:
        assert(0);
        return FALSE;
    break;
    }
    return TRUE;
}

/*******************************************************************************/
static int crc_close_r (void *reent, devoptab_t *dot )
/*******************************************************************************/
{
    crc_t *crc = dot->priv;

    if (crc) {
        /* Disable the SIMSCGC for the crc module being used*/
        *crc->simScgc |= crc->simScgcEnBit;
        /* Unhook the private crc structure and free it */
        dot->priv = NULL;
        free(crc);
        return TRUE;
    }
    else {
        return FALSE;
    }
}
/*******************************************************************************/
static long crc_write_r (void *reent, devoptab_t *dot, const void *buf, int len )
/*******************************************************************************/
{
    crc_t *crc;
    if (!dot || !dot->priv) return FALSE;
    else crc = (crc_t *) dot->priv;
    return crc->write(crc, buf, len);
}
/*******************************************************************************/
static long crc_read_r (void *reent, devoptab_t *dot, void *buf, int len )
/*******************************************************************************/
{
    crc_t *crc;
    union {
        uint32_t *u32;
        uint16_t *u16;
    } dataPtr;

    if (!dot || !dot->priv) return FALSE;
    else crc = (crc_t *) dot->priv;

    if (*crc->ctrl & CRC_CTRL_TCRC && len == 4) {               /* 32 Bit Mode */
        dataPtr.u32 = (uint32_t *)buf;
        *(dataPtr.u32) = (uint32_t)(*crc->crc);
        return 4;
    } else if (len == 2) {                                      /* 16 Bit Mode */
        dataPtr.u16 = (uint16_t *)buf;
        if ( *crc->ctrl & CRC_CTRL_TOTR1 ) {
            /* Bytes were transposed, read HU[31:240], HL[23:16] Instead*/
            *(dataPtr.u16) = (uint16_t)(*crc->crc >> 16);
        } else {
            /* Read LU[15:8], LL[7:0] Only*/
            *(dataPtr.u16) = (uint16_t)(*crc->crc & 0x0000FFFF);
        }
        return 2;
    } else {
        return 0;
    }
}

/******************************************************************************/
int crc_install(void)
/******************************************************************************/
{
    if (!(deviceInstall(DEV_MAJ_CRC, crc_open_r,  crc_ioctl, crc_close_r,
                                                        crc_write_r, crc_read_r)
                     && deviceRegister(DEVOPTAB_CRC_STR, DEV_MAJ_CRC, 0, NULL)))
        return FALSE;
    else
        return TRUE;
}

