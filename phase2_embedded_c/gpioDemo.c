/*******************************************************************************
*
* gpioDemo.c
*
* Super crude C version of the GPIO demo in phase 1.
*
* Rob Laswick
* June 18 2012
*
*******************************************************************************/
#include "kinetis.h"

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

static uint32_t dTime = 0x0007ffff;

static void delay(void)
{
    volatile uint32_t time = dTime;
    while (--time)
        ;
}

int main(void)
{
    SIM_SCFC5 |= (SIM_PORTA_ENABLE | SIM_PORTE_ENABLE);

    PORT_PCR(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT) = PORT_MUX_GPIO;
    PORT_PCR(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT) = PORT_MUX_GPIO;
    PORT_PCR(N_LED_GREEN_PORT,  N_LED_GREEN_BIT)  = PORT_MUX_GPIO;
    PORT_PCR(N_LED_BLUE_PORT,   N_LED_BLUE_BIT)   = PORT_MUX_GPIO;

    GPIOA_PDDR |= LEDS_MASK;

    for (;;) {
        delay();
        GPIOA_PSOR = LEDS_MASK;
        delay();
        GPIOA_PCOR = 1 << N_LED_ORANGE_BIT;
        delay();
        GPIOA_PCOR = 1 << N_LED_YELLOW_BIT;
        delay();
        GPIOA_PCOR = 1 << N_LED_GREEN_BIT;
        delay();
        GPIOA_PCOR = 1 << N_LED_BLUE_BIT;
    }

    return 0;
}
