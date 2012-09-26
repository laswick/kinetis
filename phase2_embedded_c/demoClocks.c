/*******************************************************************************
*
* demoClocks.c
*
* Jan Markowski
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.

*******************************************************************************/
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}


static void flashLed(int port, int pin, int numBlinks)
{
    int k;

    for (k = 0; k < numBlinks; k++) {
        gpioClear(port, pin);
        delay();
        gpioSet(port, pin);
        delay();
    }
}

int main(void) {

    int numBlinks = 10;

    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);

    for (;;) {


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
        flashLed(N_LED_BLUE_PORT, N_LED_BLUE_PIN, numBlinks);


        /* -------- 24 MHz (slow internal clock) -----------
         * Configure the Multipurpose Clock Generator output to use the internal
         * 4MHz clock locked with FLL (internal clocks can only be locked with
         * the FLL) at 24MHz.
         *
         * For FLL, the clock dividers be set before or after. (but not before 
         * PLL)
         *
         * System:   24 MHz
         * Bus:       3 MHz
         * Flexbus:   3 MHz
         * Flash:     3 MHz
         */
        clockConfigMcgOut(MCG_FLL_INTERNAL_24MHZ);
        clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_8, DIVIDE_BY_8, DIVIDE_BY_8);
        flashLed(N_LED_GREEN_PORT, N_LED_GREEN_PIN, numBlinks);
        

        /* --------  6 MHz (slow internal clock) -----------
         * With the output still at 24MHz, the dividers are quartered. Therefore,
         * the new output will be 6MHz.
         *
         * System:    6 MHz
         * Bus:       1.5 MHz
         * Flexbus:   1.5 MHz
         * Flash:     1.5 MHz
         */
        clockSetDividers(DIVIDE_BY_4, DIVIDE_BY_16, DIVIDE_BY_16, DIVIDE_BY_16);
        flashLed(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, numBlinks);
    }

    return 0;
}
