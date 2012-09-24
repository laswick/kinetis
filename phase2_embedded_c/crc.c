/*******************************************************************************
*
* crc.c
*
* Shaun Weise
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
    volatile uint32_t * gpoly32;
    volatile uint16_t * gpoly16;
    volatile uint32_t * ctrl;
    volatile uint32_t * simScgc;
    uint32_t            seed;
    unsigned            simScgcEnBit;
} crc_t;

#define CRC32       0x82608EDB
#define CRC15CAN    0x62CC
#define CRC16       0xC002
#define CRC16ARINC  0xD015

crc_t crcDflt = {
    .crc          = CRC_CRC_PTR,
    .gpoly32      = CRC_GPOLY32_PTR,
    .gpoly16      = CRC_GPOLY16_PTR,
    .ctrl         = CRC_CTRL_PTR,
    .simScgc      = SIM_SCGC6_PTR,
    .seed         = 0xA535A535,
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
static unsigned crcWrite(devoptab_t *dot, const void *data, unsigned len)
/*******************************************************************************/
{
    crc_t *crc;
    unsigned i;
    union {
        uint32_t *u32;
        uint16_t *u16;
    } dataPtr;

    if (!dot || !dot->priv) return FALSE;
    else crc = (crc_t *) dot->priv;

    if (*crc->ctrl & CRC_CTRL_TCRC) {               /* 32 Bit Mode */
        if ((len % 4) != 0) return FALSE;        /* Unaligned Data */
        dataPtr.u32 = (uint32_t *) data;
        for (i = 0; i < len/4; i++)
            *crc->crc = *(dataPtr.u32)++;
    }
    else {                                          /* 16 Bit Mode */
        if ((len % 2) != 0) return FALSE;        /* Unaligned Data */
        dataPtr.u16 = (uint16_t *) data;
        for (i = 0; i < len/2; i++)
            *crc->crc = *(dataPtr.u16)++;
    }

    return len;
}

/*******************************************************************************/
static unsigned crcRead(devoptab_t *dot, void *data, unsigned len)
/*******************************************************************************/
{
    crc_t *crc;
    union {
        uint32_t *u32;
        uint16_t *u16;
    } dataPtr;

    if (!dot || !dot->priv) return FALSE;
    else crc = (crc_t *) dot->priv;

    if (*crc->ctrl & CRC_CTRL_TCRC) {               /* 32 Bit Mode */
        dataPtr.u32 = (uint32_t *) data;
        *(dataPtr.u32) = *crc->crc;
        len = 4;
    }
    else {                                          /* 16 Bit Mode */
        dataPtr.u16 = (uint16_t *) data;
        *(dataPtr.u16) = *crc->crc;
        len = 2;
    }

    return len;
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
/* TODO: return errors if flags or cmd is bad */
{
    crc_t *crc;
    unsigned reg;

    if (!dot || !dot->priv) return FALSE;
    else crc = (crc_t *) dot->priv;

    if (cmd < 0 || cmd > MAX_IO_IOCTRL_CRC_CMDS) return FALSE;

    switch (cmd) {
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
        *crc->crc = flags;
        *crc->ctrl &= ~(CRC_CTRL_WAS);
        crc->seed = flags;
    break;
    case IO_IOCTL_CRC_GET_SEED:
        return crc->seed;
    break;
    case IO_IOCTL_CRC_SET_POLY:
        if (*crc->ctrl & CRC_CTRL_TCRC)
            *crc->gpoly32 = (uint32_t) flags;
        else
            *crc->gpoly16 = (uint16_t) flags;
    break;
    case IO_IOCTL_CRC_GET_POLY:
        if (*crc->ctrl & CRC_CTRL_TCRC)
            return (int)*crc->gpoly32;
        else
            return (int)*crc->gpoly16;
    break;
    case IO_IOCTL_CRC_SET_WIDTH:
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
    case IO_IOCTL_CRC_GET_WIDTH:
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
    /* You could just put your write function here, but I want switch between
     * polled & interupt functions here */
    return crcWrite(dot, buf, len);
}
/*******************************************************************************/
static long crc_read_r (void *reent, devoptab_t *dot, void *buf, int len )
/*******************************************************************************/
{
    /* You could just put your read function here, but I want switch between
     * polled & interupt functions here */
    return crcRead(dot, buf, len);
}

/******************************************************************************/
int crc_install(void)
/******************************************************************************/
{
    if (!(deviceInstall(DEV_MAJ_CRC, crc_open_r,  crc_ioctl, crc_close_r,
                                                        crc_write_r, crc_read_r)
                                && deviceRegister("crc", DEV_MAJ_CRC, 0, NULL)))
        return FALSE;
    else
        return TRUE;
}

