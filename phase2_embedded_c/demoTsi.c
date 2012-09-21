/*******************************************************************************
*
* demoTSI.c
*
* David Kennedy
* July 15 2012
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
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define MODE_4PIN                 /* Use touch pads on TWR-K60N512 main board */
/* #define MODE_KEYPAD */    /* Use touch pads on TWRPI-KEYPAD daughter board */

/* #define CALIBRATE */              /* Display raw TSI counts on serial port */

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

#if defined(MODE_4PIN)
static const tsiConfig_t tsiConfig = {
    .pinEnable = BIT_5 | BIT_8 | BIT_7 | BIT_9,
    .scanc = TSI_SCANC_DEFAULT,
    .prescale = 5,
    .threshold = {
        [TSI_ORANGE_INDEX] = 100,
        [TSI_YELLOW_INDEX] = 100,
        [TSI_GREEN_INDEX]  = 100,
        [TSI_BLUE_INDEX]   = 100,
    },
};
#elif defined(MODE_KEYPAD)
static const uint8_t keymap[TSI_COUNT] = {
/*   0   1   2   3   4   5   6   7 */
     1,  0,  0,  0,  0,  8,  2,  3,
/*   8   9  10  11  12  13  14  15 */
     4,  9, 11, 10, 12,  5,  6,  7,
};
#if defined(CALIBRATE)
static const tsiConfig_t tsiConfig = {
    .pinEnable = BIT_0 | BIT_5 | BIT_6 | BIT_7 | BIT_8 | BIT_9 | BIT_10
                          | BIT_11 | BIT_12 | BIT_13 | BIT_14 | BIT_15 | BIT_15,
    .scanc = TSI_SCANC_DEFAULT,
    .prescale = 5,
    .threshold = {
        [0]  =  150, /* 1 */
        [6]  =  150, /* 2 */
        [7]  =  150, /* 3 */
        [8]  =  150, /* 4 */
        [13] =  150, /* 5 */
        [14] =  150, /* 6 */
        [15] =  150, /* 7 */
        [5]  =  150, /* 8 */
        [9]  =  150, /* 9 */
        [10] =  150, /* * */
        [11] =  150, /* 0 */
        [12] =  150, /* # */
    },
};
#endif
#endif

#if defined(CALIBRATE)
int main(void)
{
    int fd;
    int pin;
    uint32_t value;
    char buf[32];

    uart_install();

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);

    tsiInit(&tsiConfig);

    fd = open("uart3", 0, 0);
    if (fd == -1) {
        assert(0);
    }
    ioctl(fd, IO_IOCTL_UART_BAUD_SET, 115200);

    for (;;) {
        for (pin=0; pin < TSI_COUNT; pin ++) {
            if (tsiConfig.pinEnable & (1 << pin)) {
                value = tsiReadRaw(pin);

                buf[0] = (pin / 10) + '0';
                buf[1] = (pin % 10) + '0';
                buf[2] = ':';
                buf[3] = ' ';
                buf[8] = (value % 10) + '0';
                value /= 10;
                buf[7] = (value % 10) + '0';
                value /= 10;
                buf[6] = (value % 10) + '0';
                value /= 10;
                buf[5] = (value % 10) + '0';
                value /= 10;
                buf[4] = (value % 10) + '0';
                value /= 10;
                buf[9] = ' ';
                buf[10] = 0;
                write(fd, buf, 10);

                gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
            }
        }
        buf[0] = '\r';
        buf[1] = '\n';
        buf[2] = 0;
        write(fd, buf, 2);
        delay();
    }
}
#else
#if defined(MODE_4PIN)
int main(void)
{
    uint32_t state, lastState = 0, pressed;

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    tsiInit(&tsiConfig);

    for (;;) {
        delay();

        state = tsiRead(&tsiConfig);

        pressed = state & ~lastState;

        if (pressed & TSI_ORANGE_BIT) {
            gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        }
        if (pressed & TSI_YELLOW_BIT) {
            gpioToggle(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        }
        if (pressed & TSI_GREEN_BIT) {
            gpioToggle(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        }
        if (pressed & TSI_BLUE_BIT) {
            gpioToggle(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        }

        lastState = state;
    }

    return 0;
}
#elif defined(MODE_KEYPAD)
int main(void)
{
    uint32_t state, lastState = 0, pressed;
    uint8_t index, key;
    int fd, len;
    char buf[2];
    tsiConfigure_t tsiConfigure;

    tsi_install();

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    fd = open("tsi0", 0, 0);
    if (fd == -1) {
        assert(0);
    }
    ioctl(fd, IO_IOCTL_TSI_SET_PRESCALE, 5);
    for (index = 0; index < TSI_COUNT; index++) {
        if (keymap[index] != 0) {
            tsiConfigure.pin = index;
            tsiConfigure.threshold = 150;

            ioctl(fd, IO_IOCTL_TSI_CONFIGURE_PIN, (int)&tsiConfigure);
        }
    }

    for (;;) {
        delay();

        len = read(fd, buf, 2);
        assert(len == 2);
        state = (buf[0] & 0xFF) | ((buf[1] & 0xFF) << 8);

        pressed = state & ~lastState;

        for (index = 0; index < TSI_COUNT; index++) {
            if (pressed & (1 << index)) {
                key = keymap[index];

                if (key & BIT_0) {
                    gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
                }
                else {
                    gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
                }

                if (key & BIT_1) {
                    gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
                }
                else {
                    gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
                }

                if (key & BIT_2) {
                    gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
                }
                else {
                    gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
                }
                if (key & BIT_3) {
                    gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
                }
                else {
                    gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
                }
            }
        }

        lastState = state;
    }

    return 0;
}
#endif
#endif
