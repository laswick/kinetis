/*******************************************************************************
*
* i2c.c
*
* David Kennedy
*
* Low-level driver for the Kinetis I2C module, in master mode.
*
* API: i2c_install()
*      Use the newlib open/close/read/write/ioctl functions.
*      Devices are named "i2c0", "i2c1".
*
* Copyright (C) 2013 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define NUM_I2C 2
#define DEFAULT_I2C_SPEED 100000

typedef struct i2cCfg_s {
    volatile i2cPort_t *port;
    uint32_t SCGC4Mask;
    uint32_t SCGC5Mask;
    uint32_t portBase;
    uint32_t sclPin;
    uint32_t sdaPin;
    uint32_t pinCfg;
} i2cCfg_t;

static const i2cCfg_t i2cCfg[NUM_I2C] = {
    {
        .port      = I2C0_REG_PTR,
        .SCGC4Mask = SIM_SCGC4_I2C0_ENABLE,
        .SCGC5Mask = SIM_SCGC5_PORTD_ENABLE,
        .portBase  = PORTD,
        .sclPin    = 8,
        .sdaPin    = 9,
        .pinCfg    = PORT_MUX_ALT2 | PORT_ODE,
    },
    {
        .port      = I2C1_REG_PTR,
        .SCGC4Mask = SIM_SCGC4_I2C1_ENABLE,
        .SCGC5Mask = SIM_SCGC5_PORTC_ENABLE,
        .portBase  = PORTC,
        .sclPin    = 10,
        .sdaPin    = 11,
        .pinCfg    = PORT_MUX_ALT2 | PORT_ODE,
    },
};

typedef struct i2cPriv_s {
    bool           open;
    uint8_t        address;
    uint32_t       speed;
    const i2cCfg_t *cfg;
} i2cPriv_t;

static i2cPriv_t i2cPriv[NUM_I2C];

/******************************************************************************/
/* See Table 50-41 I2C Divider and Hold Values */
uint16_t sclDivTab[] = {
    /* 00 */   20, /* 01 */   22, /* 02 */   24, /* 03 */   26,
    /* 04 */   28, /* 05 */   30, /* 06 */   34, /* 07 */   40,
    /* 08 */   28, /* 09 */   32, /* 0A */   36, /* 0B */   40,
    /* 0C */   44, /* 0D */   48, /* 0E */   56, /* 0F */   68,

    /* 10 */   48, /* 11 */   56, /* 12 */   64, /* 13 */   72,
    /* 14 */   80, /* 15 */   88, /* 16 */  104, /* 17 */  128,
    /* 18 */   80, /* 19 */   96, /* 1A */  112, /* 1B */  128,
    /* 1C */  144, /* 1D */  160, /* 1E */  192, /* 1F */  240,

    /* 20 */  160, /* 21 */  192, /* 22 */  224, /* 23 */  256,
    /* 24 */  288, /* 25 */  320, /* 26 */  384, /* 27 */  480,
    /* 28 */  320, /* 29 */  384, /* 2A */  448, /* 2B */  512,
    /* 2C */  576, /* 2D */  640, /* 2E */  768, /* 2F */  960,

    /* 30 */  640, /* 31 */  768, /* 32 */  896, /* 33 */ 1024,
    /* 34 */ 1152, /* 35 */ 1280, /* 36 */ 1536, /* 37 */ 1920,
    /* 38 */ 1280, /* 39 */ 1536, /* 3A */ 1792, /* 3B */ 2048,
    /* 3C */ 2304, /* 3D */ 2560, /* 3E */ 3072, /* 3F */ 3840,
};

static int i2cSetClock(volatile i2cPort_t *port, uint32_t speed)
{
    uint32_t busClk;
    uint32_t i2cClk;
    uint32_t error;
    uint32_t bestError = 0xFFFF;
    uint8_t bestIcr = 0xFF;
    uint8_t icr;
    uint16_t slt;

    busClk = clockGetFreq(CLOCK_BUS);

    for (icr = 0; icr < sizeof(sclDivTab) / sizeof(sclDivTab[0]); icr++) {
        i2cClk = busClk / sclDivTab[icr];
        if (i2cClk > speed) {
            i2cClk /= 2;
            if (i2cClk > speed) {
                i2cClk /= 2;
                if (i2cClk > speed) {
                    continue;
                }
            }
        }
        error = speed - i2cClk;
        if (error < bestError) {
            bestError = error;
            bestIcr = icr;
        }
    }
    if (bestIcr == 0xFF) {
        return FALSE;
    }

    icr = bestIcr;
    i2cClk = busClk / sclDivTab[bestIcr];
    if (i2cClk > speed) {
        i2cClk /= 2;
        if (i2cClk > speed) {
            i2cClk /= 2;
            icr |= I2C_F_MULT_4;
        }
        else {
            icr |= I2C_F_MULT_2;
        }
    }

    port->c1 = 0;

    port->f = icr;
    slt = sclDivTab[bestIcr] / 2 + 1;
    port->slth = slt >> 8;
    port->sltl = slt;

    port->c1 = I2C_C1_IICEN;

    return TRUE;
}

static int i2cMasterInit(const i2cCfg_t *cfg, uint32_t speed)
{
    volatile i2cPort_t *port = cfg->port;

    /* Enable clock gate for I2C module */
    SIM_SCGC4 |= cfg->SCGC4Mask;

    /* Configure pin multiplexer for I2C SDA and SCL pins */
    SIM_SCGC5 |= cfg->SCGC5Mask;
    PORT_PCR(cfg->portBase, cfg->sclPin) = cfg->pinCfg;
    PORT_PCR(cfg->portBase, cfg->sdaPin) = cfg->pinCfg;

    /* Select and set clock divider */
    if (!i2cSetClock(port, speed)) {
        return FALSE;
    }

    port->c1 = I2C_C1_IICEN;

    return TRUE;
}

static void i2cMasterClose(const i2cCfg_t *cfg)
{
    volatile i2cPort_t *port = cfg->port;

    /* Disable I2C */
    port->c1 = 0;
    /* Disable clock gate for I2C module */
    SIM_SCGC4 &= ~(cfg->SCGC4Mask);
}

static uint8_t i2cWaitStatus(volatile i2cPort_t *port, uint8_t mask,
                                                       uint8_t value)
{
    uint8_t status;
    uint32_t timeout = 5000;
    for (;;) {
        status = port->s;
        if ((status & mask) == value) {
            port->s = status & (I2C_S_IICIF | I2C_S_ARBL);
            return status;
        }
        if (--timeout == 0) {
            return status;
        }
    }
}

static uint32_t i2cMasterWriteRead(volatile i2cPort_t *port, uint8_t address,
                            const uint8_t *txData, uint32_t txLength,
                                  uint8_t *rxData, uint32_t rxLength)

{
    uint8_t status;
    uint32_t txIndex = 0;
    uint32_t rxIndex = 0;

    if (txLength) {
        port->c1 |= I2C_C1_TX;
        port->c1 |= I2C_C1_MST;

        status = i2cWaitStatus(port, I2C_S_BUSY, I2C_S_BUSY);
        if (!(status & I2C_S_BUSY)) {
            goto i2cend;
        }

        port->d = (address << 1);

        status = i2cWaitStatus(port, I2C_S_IICIF, I2C_S_IICIF);
        if (!(status & I2C_S_IICIF)
         ||  (status & (I2C_S_RXAK | I2C_S_ARBL))) {
            goto i2cend;
        }

        for (txIndex = 0; txIndex < txLength; txIndex++) {
            port->d = txData[txIndex];

            status = i2cWaitStatus(port, I2C_S_IICIF, I2C_S_IICIF);
            if (!(status & I2C_S_IICIF)
             ||  (status & (I2C_S_RXAK))) {
                goto i2cend;
            }
        }

        if (rxLength) {
            port->c1 |= I2C_C1_RSTA;

            status = i2cWaitStatus(port, I2C_S_BUSY, I2C_S_BUSY);
            if (!(status & I2C_S_BUSY)) {
                goto i2cend;
            }
        }
        else {
            port->c1 &= ~I2C_C1_MST;

            status = i2cWaitStatus(port, I2C_S_BUSY, 0);
            if (status & I2C_S_BUSY) {
                goto i2cend;
            }
        }
    }

    if (rxLength) {
        port->c1 |= I2C_C1_TX;
        port->c1 |= I2C_C1_MST;

        status = i2cWaitStatus(port, I2C_S_BUSY, I2C_S_BUSY);
        if (!(status & I2C_S_BUSY)) {
            goto i2cend;
        }

        port->d = (address << 1) | 0x01;

        status = i2cWaitStatus(port, I2C_S_IICIF, I2C_S_IICIF);
        if (!(status & I2C_S_IICIF)
         ||  (status & (I2C_S_RXAK | I2C_S_ARBL))) {
            goto i2cend;
        }

        port->c1 &= ~I2C_C1_TX;
        if (rxLength == 1) {
            port->c1 |= I2C_C1_TXAK;
        }
        else {
            port->c1 &= ~I2C_C1_TXAK;
        }

        rxData[0] = port->d; /* Dummy read to start transfer */

        for (rxIndex = 0; rxIndex < rxLength; rxIndex++) {
            status = i2cWaitStatus(port, I2C_S_IICIF, I2C_S_IICIF);
            if (!(status & I2C_S_IICIF)) {
                goto i2cend;
            }
            if (rxIndex == rxLength - 1) {
                port->c1 &= ~I2C_C1_MST;

                status = i2cWaitStatus(port, I2C_S_BUSY, 0);
                if (status & I2C_S_BUSY) {
                    goto i2cend;
                }
            }
            else if (rxIndex == rxLength - 2) {
                port->c1 |= I2C_C1_TXAK;
            }
            rxData[rxIndex] = port->d;
        }
    }

i2cend:
    port->c1 &= ~(I2C_C1_MST | I2C_C1_TX | I2C_C1_TXAK);
    return txIndex + rxIndex;
}

/******************************************************************************/
static int i2c_open_r(void *reent, devoptab_t *dot, int mode, int flags)
{
    i2cPriv_t *priv = (i2cPriv_t *)dot->priv;
    if (priv->open) {
        return FALSE;
    }

    priv->address = 0;
    priv->speed = 100000;
    if (i2cMasterInit(priv->cfg, priv->speed)) {
        priv->open = TRUE;
        return TRUE;
    }
    return FALSE;
}

/******************************************************************************/
static int i2c_ioctl(devoptab_t *dot, int cmd, int flags)
{
    i2cPriv_t *priv = (i2cPriv_t *)dot->priv;

    if (!priv->open) {
        return FALSE;
    }

    switch(cmd) {
    case IO_IOCTL_I2C_SET_SPEED:
        if (!i2cSetClock(priv->cfg->port, flags)) {
            return FALSE;
        }
        priv->speed = flags;
        return TRUE;

    case IO_IOCTL_I2C_GET_SPEED:
        return priv->speed;

    case IO_IOCTL_I2C_SET_ADDRESS:
        priv->address = flags & 0x7F;
        return TRUE;

    case IO_IOCTL_I2C_GET_ADDRESS:
        return priv->address;

    default:
        assert(0);
        return FALSE;
    }

}
/******************************************************************************/
static int i2c_close_r(void *reent, devoptab_t *dot)
{
    i2cPriv_t *priv = (i2cPriv_t *)dot->priv;
    if (priv->open) {
        priv->open = FALSE;

        i2cMasterClose(priv->cfg);
        return TRUE;
    }
    else {
        return FALSE;
    }
}

/******************************************************************************/
static long i2c_write_r(void *reent, devoptab_t *dot, const void *buf, int len)
{
    i2cPriv_t *priv = (i2cPriv_t *)dot->priv;

    return i2cMasterWriteRead(priv->cfg->port, priv->address,
                                               (const uint8_t *)buf, len, 0, 0);
}

/******************************************************************************/
static long i2c_read_r(void *reent, devoptab_t *dot, void *buf, int len)
{
    i2cPriv_t *priv = (i2cPriv_t *)dot->priv;

    return i2cMasterWriteRead(priv->cfg->port, priv->address,
                                                     0, 0, (uint8_t *)buf, len);
}

/******************************************************************************/
int i2c_install(void)
{
    int ret = TRUE;

    i2cPriv[0].open =
    i2cPriv[1].open = FALSE;
    i2cPriv[0].cfg = &i2cCfg[0];
    i2cPriv[1].cfg = &i2cCfg[1];

    if (!deviceInstall(DEV_MAJ_I2C, i2c_open_r, i2c_ioctl, i2c_close_r,
                                                     i2c_write_r, i2c_read_r)) {
        ret = FALSE;
    }
    if (!deviceRegister("i2c0", DEV_MAJ_I2C, 0, &i2cPriv[0])) {
        ret = FALSE;
    }
    if (!deviceRegister("i2c1", DEV_MAJ_I2C, 1, &i2cPriv[1])) {
        ret = FALSE;
    }
    return ret;
}
