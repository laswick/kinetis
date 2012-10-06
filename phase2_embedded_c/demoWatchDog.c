/*******************************************************************************
*
* demoWatchdog.c
*
* Andrew Krajewski
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
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

int main(void)
{
    watchDogDisable(); /* while configuring non-safety critical portion */

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_SWITCH_1_PORT,   N_SWITCH_1_PIN,   GPIO_INPUT);

    delay(); delay(); delay(); delay(); delay(); delay(); delay(); delay();
    gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    delay(); delay(); delay(); delay(); delay(); delay(); delay(); delay();
    gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    delay(); delay(); delay(); delay(); delay(); delay(); delay(); delay();
    gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
    delay(); delay(); delay(); delay(); delay(); delay(); delay(); delay();
    gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

    /* Initialize watchDog */
//  watchDogConfig();
    watchDogInit();

    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

        delay();
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        delay();
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        delay();
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

	/* While no button pressed smack the watchdog around */
	if (gpioRead(N_SWITCH_1_PORT, N_SWITCH_1_PIN) != 0) {
            watchDogKick();
	}
        else {
            int test = 5;
            test++;
        }
/*
	if (sw2 pressed) {
            watchDogDisable();
	}
*/
    }

    return 0;
}
