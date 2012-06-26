/*******************************************************************************
*
* demoGpio.c
*
* Rob Laswick
* June 19 2012
*
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define N_LED_ORANGE_PORT PORTA
#define N_LED_ORANGE_PIN  11

#define N_LED_YELLOW_PORT PORTA
#define N_LED_YELLOW_PIN  28

#define N_LED_GREEN_PORT  PORTA
#define N_LED_GREEN_PIN   29

#define N_LED_BLUE_PORT   PORTA
#define N_LED_BLUE_PIN    10

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

int main(void)
{
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

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
    }

    return 0;
}
