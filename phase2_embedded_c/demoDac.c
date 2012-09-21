/*******************************************************************************
*
* demoDAC.c
*
* Daryl Hillman
* Sept 2012
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

static uint16_t dacData[DAC_DATA_MAX] = {
     0x0555, 0x0666, 0x0777, 0x0888, 0x0999, 0x0aaa, 0x0bbb, 0x0fc0,
     0x0bbb, 0x0aaa, 0x0999, 0x0888, 0x0777, 0x0666, 0x0555, 0x0000,
};

static void pit0ISR(void)
{
    static int toggle = 0;
    if (toggle)
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    else
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    toggle ^= 0x1;
    dac0->c0 = DAC_CR0_TRGF;
    pitCtrl->pit[PIT_0].flags = 1;
}

static void pit1ISR(void)
{
    static int toggle = 0;
    if (toggle)
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    else
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    toggle ^= 0x1;
    pitCtrl->pit[PIT_1].flags = 1;
}

static void pit2ISR(void)
{
    static int toggle = 0;
    if (toggle)
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
    else
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
    toggle ^= 0x1;
    pitCtrl->pit[PIT_2].flags = 1;
}

static void pit3ISR(void)
{
    static int toggle = 0;
    if (toggle)
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
    else
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
    toggle ^= 0x1;
    pitCtrl->pit[PIT_3].flags = 1;
}

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}


int main(void)
{
    int i;
    uint32_t count = 25000000;

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);
//    gpioConfig(N_SWITCH_0_PORT, N_SWITCH_0_PIN, GPIO_INPUT | GPIO_PULLDOWN);
//    gpioConfig(N_SWITCH_1_PORT, N_SWITCH_1_PIN, GPIO_INPUT | GPIO_PULLDOWN);

    dac0Init();

    dac0->c0 = DAC_CR0_FLAG;
    dac0->c1 = DAC_CR1_FLAG;
    dac0->c2 = DAC_CR2_FLAG;

    for (i = 0; i < DAC_DATA_MAX; i++) {
        dac0->data[i].low  = (uint8_t) dacData[i];
	dac0->data[i].high = (uint8_t)(dacData[i] >> 8);
    }

    pitInit(PIT_0, pit0ISR, count);
    pitInit(PIT_1, pit1ISR, count/2);
    pitInit(PIT_2, pit2ISR, count/3);
    pitInit(PIT_3, pit3ISR, count/4);

    while (1) {
//	if (gpioRead(N_SWITCH_1_PORT, N_SWITCH_1_PIN))
//            count += 25000;
//        else if (gpioRead(N_SWITCH_0_PORT, N_SWITCH_0_PIN))
//            count -= 25000;

	for (i = PIT_0; i < MAX_PIT; i++) {
            pitCtrl->pit[i].loadVal = count / (1+i);
	}
	delay();
    }
}







