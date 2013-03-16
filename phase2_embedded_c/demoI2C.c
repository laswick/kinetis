/*******************************************************************************
*
* demoI2C.c
*
* David Kennedy
*
* March 2013
*
* Copyright (C) 2013 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"
#include "util.h"

static void setClock(void)
{
    /* -------- 100 MHz (external clock) -----------
     * Configure the Multipurpose Clock Generator output to use the external
     * clock locked with a PLL at the maximum frequency of 100MHZ
     *
     * For PLL, the dividers must be set first.
     *
     * System:  100 MHz
     * Bus:      50 MHz
     * Flexbus:  50 MHz
     * Flash:    25 MHz
     */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

    return;
}

#if 0
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

static void i2cSetClock(volatile i2cPort_t *port, uint32_t speed)
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
        return; /* TODO: ??? */
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
    port->f = icr;

    slt = sclDivTab[bestIcr] / 2 + 1;
    port->slth = slt >> 8;
    port->sltl = slt;
}

static void i2cMasterInit(uint32_t speed)
{
    volatile i2cPort_t *port = I2C0_REG_PTR;

    SIM_SCGC4 |= SIM_SCGC4_I2C0_ENABLE;

    SIM_SCGC5 |= SIM_SCGC5_PORTD_ENABLE;
    PORT_PCR(PORTD, 8) = PORT_MUX_ALT2 | PORT_ODE;
    PORT_PCR(PORTD, 9) = PORT_MUX_ALT2 | PORT_ODE;

    i2cSetClock(port, speed);

    port->c1 = I2C_C1_IICEN;
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

static uint32_t i2cMasterWriteRead(uint8_t address,
                            const uint8_t *txData, uint32_t txLength,
                                  uint8_t *rxData, uint32_t rxLength)

{
    volatile i2cPort_t *port = I2C0_REG_PTR;
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

#endif

#define ACCEL_ADDR 0x4C

int main(void)
{
    uint8_t buffer[16];
    int8_t x, y, z;
    uint32_t len;
    uint32_t init = 0;
    int i2c;

    setClock();

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_HIGH);

    i2c_install();

    i2c = open("i2c0", 0, 0);
    if (i2c == -1) {
        assert(0);
    }
    ioctl(i2c, IO_IOCTL_I2C_SET_SPEED, 400000);
    ioctl(i2c, IO_IOCTL_I2C_SET_ADDRESS, ACCEL_ADDR);

    for (;;) {
        delay();

        if (!init) {
            buffer[0] = 0x07; /* Mode Register */
            buffer[1] = 0x01; /* Active */
            len = write(i2c, buffer, 2);
            if (len != 2) {
                continue;
            }
        }

        buffer[0] = 0x00;
        len = write(i2c, buffer, 1);
        if (len != 1) {
            init = 0;
            continue;
        }
        len = read(i2c, buffer, 3);
        if (len != 3) {
            init = 0;
            continue;
        }
        init = 1;

        x = (int8_t)(buffer[0] << 2) >> 2;
        y = (int8_t)(buffer[1] << 2) >> 2;
        z = (int8_t)(buffer[2] << 2) >> 2;

        if (x < -5) {
            gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        } else {
            gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        }

        if (x > -15 && x < 15) {
            gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        } else {
            gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        }

        if (x > 5) {
            gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        } else {
            gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        }

        gpioToggle(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
    }
}
