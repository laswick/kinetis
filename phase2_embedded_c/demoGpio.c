/*******************************************************************************
*
* demoGpio.c
*
* Rob Laswick
* June 19 2012
*
*******************************************************************************/
/*
 * standard headers
 * processors headers
 * operating system headers
 * custom headers
 */

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define N_LED_ORANGE_PORT PORTA
#define N_LED_ORANGE_BIT  11

#define N_LED_YELLOW_PORT PORTA
#define N_LED_YELLOW_BIT  28

#define N_LED_GREEN_PORT  PORTA
#define N_LED_GREEN_BIT   29

#define N_LED_BLUE_PORT   PORTA
#define N_LED_BLUE_BIT    10

#define LEDS_MASK  (1 << N_LED_ORANGE_BIT) | (1 << N_LED_YELLOW_BIT) \
                 | (1 << N_LED_GREEN_BIT)  | (1 << N_LED_BLUE_BIT)

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

int main(void)
{
    assert(0);                          /* test ... worth while demonstrating */
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_BIT,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_BIT,   GPIO_OUTPUT | GPIO_LOW);

    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT);
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT);
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_BIT);
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_BIT);

        delay();
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT);
        delay();
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT);
        delay();
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_BIT);
        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_BIT);
    }

    return 0;
}
