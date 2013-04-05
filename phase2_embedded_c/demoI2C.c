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
