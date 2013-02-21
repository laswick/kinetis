/*******************************************************************************
*
* demoMessageQueue.c
*
* Jan Markowski
*
* This is a simple demo that shows message queues in action. It utilizes each 
* of the 4 LEDs, each one representing a task. The ORANGE LED blinks each 
* time a message is posted into the mailbox. The other 3 LEDs wait to fetch 
* the mail and blink the total number of times that a message has been posted
* by the ORANGE LED. 
* Whenever the counting LEDs finish counting, they "rest" for some duration 
* before they fetch another message. If the fetching is timed-out, the LED
* remains ON for 1 second. Each of the 3 LEDs count at different speeds. 
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

#define MSG_TIMEOUT        MS2ST(1000)
#define MSG_POST_PERIOD    2000
#define REST_FROM_COUNTING 3000
#define COUNT_SPEED_YELLOW 200
#define COUNT_SPEED_GREEN  400
#define COUNT_SPEED_BLUE   800
#define LED_TIME_ON        200

/*
 * Message queue declaration.
 */
#define MAILBOX_SIZE 5

/* 
 * Static working areas can also be used for temporary buffers and not just 
 * threads. 
 * e.g. static WORKING_AREA(buffer, 128);
 */
static msg_t buffer[MAILBOX_SIZE];
static MAILBOX_DECL(msgQ, buffer, MAILBOX_SIZE);

void assert_(const char *file, const int line) { }

/* 
 * Working areas for the three LED flashing threads.
 */
static WORKING_AREA(waThread1, THREADS_STACK_SIZE);
static WORKING_AREA(waThread2, THREADS_STACK_SIZE);
static WORKING_AREA(waThread3, THREADS_STACK_SIZE);

static msg_t Thread1(void *arg) {
	msg_t msg;
	msg_t status;
	int k;

    while (TRUE) {
		status = chMBFetch(&msgQ, &msg, MSG_TIMEOUT);

		if (status == RDY_OK) {

			for (k = 0; k < msg; k++) {
        		chThdSleepMilliseconds(COUNT_SPEED_YELLOW);
        		gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        		chThdSleepMilliseconds(LED_TIME_ON);
        		gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
			}
        	chThdSleepMilliseconds(REST_FROM_COUNTING);

		} else if (status == RDY_TIMEOUT) {

        	chThdSleepMilliseconds(1000);
        	gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

		}
    }

    return 0;
}

static msg_t Thread2(void *arg) {
	msg_t msg;
	msg_t status;
	int k;

    while (TRUE) {
		status = chMBFetch(&msgQ, &msg, MSG_TIMEOUT);

		if (status == RDY_OK) {

			for (k = 0; k < msg; k++) {
        		chThdSleepMilliseconds(COUNT_SPEED_GREEN);
        		gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        		chThdSleepMilliseconds(LED_TIME_ON);
        		gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
			}
        	chThdSleepMilliseconds(REST_FROM_COUNTING);

		} else if (status == RDY_TIMEOUT) {

        	chThdSleepMilliseconds(1000);
        	gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);

		}
    }

    return 0;
}

static msg_t Thread3(void *arg) {
	msg_t msg;
	msg_t status;
	int k;

    while (TRUE) {
		status = chMBFetch(&msgQ, &msg, MSG_TIMEOUT);

		if (status == RDY_OK) {

			for (k = 0; k < msg; k++) {
        		chThdSleepMilliseconds(COUNT_SPEED_BLUE);
        		gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        		chThdSleepMilliseconds(LED_TIME_ON);
        		gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
			}
        	chThdSleepMilliseconds(REST_FROM_COUNTING);

		} else if (status == RDY_TIMEOUT) {

        	chThdSleepMilliseconds(1000);
        	gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

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
	int counter; 

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
	 * Initialize the mailbox with 4 buffer elements.
	 */
	chMBInit(&msgQ, (msg_t *) &buffer, MAILBOX_SIZE);

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

	/*
	 * Send an ever-incrementing counter to the mailbox. 
	 */
	counter = 0;
    while (TRUE) {
		counter++;

       	chThdSleepMilliseconds(MSG_POST_PERIOD);

		status = chMBPost(&msgQ, counter, MSG_TIMEOUT);

		if (status == RDY_OK) {

        	gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        	chThdSleepMilliseconds(LED_TIME_ON);
        	gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);

		} else if (status == RDY_TIMEOUT) {

			/* 
			 * If timed out, just leave the LED on... 
			 */
			gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
		}
    }

    return 0;
}
