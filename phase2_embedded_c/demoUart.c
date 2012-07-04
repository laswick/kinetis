/*******************************************************************************
*
* demoUart.c
*
* James McAnanama
* Canada Day 2012
*
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"
#include "util.h"

/* Power on default clocks */
#define SYSTEM_CLOCK_HZ  20480000
#define    BUS_CLOCK_HZ  20480000
#define COLOUR_STRINGS " ORANGE ", " YELLOW ", " GREEN ", " BLUE "
#define COLOUR_STRLEN         8,        8,       7,      6
int main(void)
{
    static char *colourStrings[] = { COLOUR_STRINGS };
    static const  int32_t colourStrlen[]       = { COLOUR_STRLEN };
    int32_t uartUp = FALSE;
    uartIF_t uart = {
        .uart             = UART3,
        .systemClockHz    = SYSTEM_CLOCK_HZ,
        .busClockHz       = BUS_CLOCK_HZ,
        .baud             = 115200,
        .responseWaitTime = 250,
    };
    uartIF_t *uartIF = &uart;

    if  (uartInit(uartIF) != ERROR) {
        uartUp = TRUE;
    }
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        uartWrite(uartIF, (uint8_t *) colourStrings[0], colourStrlen[0]);
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        uartWrite(uartIF, (uint8_t *) colourStrings[1], colourStrlen[1]);
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        uartWrite(uartIF, (uint8_t *) colourStrings[2], colourStrlen[2]);
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        uartWrite(uartIF, (uint8_t *) colourStrings[3], colourStrlen[3]);

        delay();
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        delay();
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        delay();
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
    }

    uartFree(uartIF);
    return 0;
}
