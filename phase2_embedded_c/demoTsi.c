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
#include "util.h"

/*******************************************************************************
* TSI Demo Configuration
*
* TSI_MODE: Selects the TSI configuration:
*   TSI_MODE_4PIN:    Use the touch pads on the TWR-K60N512 main board.
*                     Uses the raw TSI API.
*   TSI_MODE_KEYPAD:  Use the touch pads on the TWRPI-KEYPAD duaghter board.
*                     Uses the POSIX TSI API.
*
* CALIBRATE:
*   TRUE:  Display the raw TSI counts on the serial port (UART3 115200 8N1).
*   FALSE: Toggle the LEDs based on TSI inputs.
*******************************************************************************/
#define TSI_MODE_4PIN   1
#define TSI_MODE_KEYPAD 2

#define TSI_MODE  TSI_MODE_4PIN
#define CALIBRATE FALSE

#if TSI_MODE == TSI_MODE_4PIN
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
#elif TSI_MODE == TSI_MODE_KEYPAD
/*******************************************************************************
* Keypad layout:    TSI Inputs:        LED Output:
*
*   1  2  3           0  6  7            1  2  3
*   4  5  6           8 13 14            4  5  6
*   7  8  9          15  5  9            7  8  9
*   *  0  #          10 11 12           11 10 12
*******************************************************************************/
static const uint8_t keymap[TSI_COUNT] = {
/*   0   1   2   3   4   5   6   7 */
     1,  0,  0,  0,  0,  8,  2,  3,
/*   8   9  10  11  12  13  14  15 */
     4,  9, 11, 10, 12,  5,  6,  7,
};
#if CALIBRATE
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

#if CALIBRATE
/*******************************************************************************
* CALIBRATE mode demo.
*
* Periodically scans the enabled TSI inputs and displays the raw "count" value
* for each electrode. Use this mode to determine appropriate values to use
* for the threshold.
*******************************************************************************/
int main(void)
{
    int fd;
    int pin;
    uint32_t value;

    setClock();

    uart_install();
    fd = fdevopen(stdin,  "uart3", 0, 0);
    ioctl(fd, IO_IOCTL_UART_BAUD_SET, 115200);
    assert(fd != -1);
    fd = fdevopen(stdout, "uart3", 0, 0);
    assert(fd != -1);
    fd = fdevopen(stderr, "uart3", 0, 0);
    assert(fd != -1);

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);

    tsiInit(&tsiConfig);

    for (;;) {
        for (pin=0; pin < TSI_COUNT; pin ++) {
            if (tsiConfig.pinEnable & (1 << pin)) {
                value = tsiReadRaw(pin);

                printf("%2d: %5d ", pin, value);
            }
        }
        printf("\r\n");
        delay();
        delay();
        gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    }
}
#else

#if TSI_MODE == TSI_MODE_4PIN
/*******************************************************************************
* 4PIN mode demo.
*
* When a key is pressed, the corresponding LED is toggled.
*
* This demo uses the raw API to access TSI.
*******************************************************************************/
int main(void)
{
    uint32_t state, lastState = 0, pressed;

    setClock();

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

#elif TSI_MODE == TSI_MODE_KEYPAD
/*******************************************************************************
* KEYPAD mode demo.
*
* When a square on the keypad is pressed, displays the binary value of that
* key on the LEDs. See the keymap above.
*
* This demo uses the POSIX API to access TSI.
*******************************************************************************/
int main(void)
{
    uint32_t state, lastState = 0, pressed;
    uint8_t index, key;
    int fd, len;
    char buf[2];
    tsiConfigure_t tsiConfigure;

    setClock();

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
