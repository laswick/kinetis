/*******************************************************************************
*
* demoSemaphore.c
*
* Jan Markowski
*
* This is a simple demo that demonstrates the use of binary and counting
* semaphores. Four semaphores are used to signal each of the other four
* LEDs which are running as their own tasks. The binary semaphore is used
* to signal a directionality change of counting. 
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include "ch.h"
#include "kinetis.h"
#include "hardware.h"

#define THREADS_STACK_SIZE 128

/*
 * Counting semaphore declarations.
 */
static SEMAPHORE_DECL(countingSem0, 1);
static SEMAPHORE_DECL(countingSem1, 0);
static SEMAPHORE_DECL(countingSem2, 0);
static SEMAPHORE_DECL(countingSem3, 0);

/* 
 * Binary semaphore declaration.
 */
static BSEMAPHORE_DECL(binarySem, FALSE); /* FALSE = initial state is 
											 -not taken- */

void assert_(const char *file, const int line) { }

/* 
 * Working areas for the three LED flashing threads.
 */
static WORKING_AREA(waThread1, THREADS_STACK_SIZE);
static WORKING_AREA(waThread2, THREADS_STACK_SIZE);
static WORKING_AREA(waThread3, THREADS_STACK_SIZE);

static int counter; 

static msg_t Thread1(void *arg) {
	msg_t status;

    while (TRUE) {
		status = chSemWait(&countingSem1);

		if (status == RDY_OK) {
        	gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        	chThdSleepMilliseconds(counter);
        	gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

			/* Signal task 0 or task 2, depending on binary sem */
			status = chBSemWaitTimeout(&binarySem, TIME_IMMEDIATE);
			if (status == RDY_OK) {
				chSemSignal(&countingSem2);
			} else if (status == RDY_TIMEOUT) {
				chBSemSignal(&binarySem);
				chSemSignal(&countingSem0);
			}
		}
    }

    return 0;
}

static msg_t Thread2(void *arg) {
	msg_t status;

    while (TRUE) {
		status = chSemWait(&countingSem2);

		if (status == RDY_OK) {
        	gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        	chThdSleepMilliseconds(counter);
        	gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);

			/* Signal task 1 or task 3, depending on binary sem */
			status = chBSemWaitTimeout(&binarySem, TIME_IMMEDIATE);
			if (status == RDY_OK) {
				chSemSignal(&countingSem1);
			} else if (status == RDY_TIMEOUT) {
				chBSemSignal(&binarySem);
				chSemSignal(&countingSem3);
			}
		}
    }

    return 0;
}

static msg_t Thread3(void *arg) {
	msg_t status;

    while (TRUE) {
		status = chSemWait(&countingSem3);

		if (status == RDY_OK) {
        	gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        	chThdSleepMilliseconds(counter);
        	gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

			/* Signal task 0 */
			chSemSignal(&countingSem2);
		}
    }

    return 0;
}

static void clocksInit(void)
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
}

static void systickInit(void)
{
    uint32_t freq = clockGetFreq(CLOCK_CORE);

    NVIC_SYSTICK_RELOAD = freq / CH_FREQUENCY - 1;
    NVIC_SYSTICK_VALUE = 0;
    NVIC_SYSTICK_CONTROL = NVIC_SYSTICK_CONTROL_ENABLE
                         | NVIC_SYSTICK_CONTROL_TICKINT
                         | NVIC_SYSTICK_CONTROL_CLKSOURCE;
}

int main(void)
{
	msg_t status;

    clocksInit();
    systickInit();
	
	/* 
	 * Initialize ChibiOS/RT. Two threads are spawned by default: IDLE and MAIN.
	 *
	 * ChibiOS supports multiple threads at the same priority level and schedules
	 * them using an "aggressive" round-robin strategy. The strategy is defined
	 * as "aggressive" because any scheduling event causes the round-robin threads
	 * to rotate.
	 */
    chSysInit();

    hwInterruptsEnable();

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_HIGH);

	/* 
	 * Initialize the counting semaphore with a count = 0, except for the 
	 * first counting semaphore to start the chain of blinking LEDs...
	 */
	chSemInit(&countingSem0, 1);
	chSemInit(&countingSem1, 0);
	chSemInit(&countingSem2, 0);
	chSemInit(&countingSem3, 0);

	/* 
	 * Create a thread that is statically allocated in memory at compile time.
	 */
    chThdCreateStatic(waThread1, 
					  sizeof(waThread1),
					  NORMALPRIO,        /* Initial priority */
					  Thread1, 			 /* Thread function */
					  NULL); 			 /* Thread arguments */
    chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL); 
    chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL); 

	chBSemInit(&binarySem, FALSE);

    while (TRUE) {
		status = chSemWait(&countingSem0);

		counter++;

		if (status == RDY_OK) {
			/* Blink LED for sem count duration */
        	gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
       		chThdSleepMilliseconds(counter);
        	gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);

			/* Signal task 1 */
			chSemSignal(&countingSem1);
		}
    }

    return 0;
}
