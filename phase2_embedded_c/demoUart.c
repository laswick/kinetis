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
#define COLOUR_STRINGS "ORANGE\t", "YELLOW\t", "GREEN\t", "BLUE\n"
enum {
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
};
int main(void)
{
    static char *colourStrings[] = { COLOUR_STRINGS };
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

    if (uartUp) {
        int i;
        uint8_t countDown = '5';
        uint8_t cr        = '\n';

        /* uartPrint is used to send a null terminated string */
        uartPrint(uartIF, "The start of something good...\n");
        delay();

        for (i = 0; i < 5; i++) {
            /* uartWrite is used to spew aritray uint8_t's our your uart */
            uartWrite(uartIF, &cr, 1);
            uartWrite(uartIF, &countDown, 1);
            uartWrite(uartIF, &cr, 1);
            delay();
            countDown--;
        }
        uartPrint(uartIF, "==========================================\n");
        delay();
    }

    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        if (uartUp) {
            uartPrint(uartIF, colourStrings[ORANGE]);
        }
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        if (uartUp) {
            uartPrint(uartIF, colourStrings[YELLOW]);
        }
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        if (uartUp) {
            uartPrint(uartIF, colourStrings[GREEN]);
        }
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        if (uartUp) {
            uartPrint(uartIF, colourStrings[BLUE]);
        }

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
