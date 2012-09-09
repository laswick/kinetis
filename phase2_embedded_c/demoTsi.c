/*******************************************************************************
*
* demoTSI.c
*
* David Kennedy
* July 15 2012
*
*******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define MODE_4PIN
//#define MODE_KEYPAD

#define CALIBRATE

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

#if defined(MODE_4PIN)
const tsiConfig_t tsiConfig = {
    .pinEnable = BIT_5 | BIT_8 | BIT_7 | BIT_9,
    .scanc = TSI_SCANC_DEFAULT,
    .prescale = 5,
    .threshold = {
        [5] = 500,
        [8] = 500,
        [7] = 500,
        [9] = 600,
    },
};
#elif defined(MODE_KEYPAD)
#endif

#if defined(CALIBRATE)
char hexChar(int nibble)
{
    if (nibble < 10) {
        return nibble + '0';
    } else {
        return nibble - 10 + 'A';
    }
}
int main(void)
{
    int fd;
    int pin;
    uint32_t value;
    char buf[32];

    uart_install();

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);

    tsiInit(&tsiConfig);

    fd = open("uart3", 0, 0);
    if (fd == -1) {
        assert(0);
    }
    ioctl(fd, IO_IOCTL_UART_BAUD_SET, 115200);

    for (;;) {
        for (pin=0; pin < TSI_COUNT; pin ++) {
            if (tsiConfig.pinEnable & (1 << pin)) {
                delay();
                value = tsiReadRaw(pin);

                buf[0] = (pin / 10) + '0';
                buf[1] = (pin % 10) + '0';
                buf[2] = ':';
                buf[3] = ' ';
                buf[4] = hexChar((value >> 12) & 0xF);
                buf[5] = hexChar((value >>  8) & 0xF);
                buf[6] = hexChar((value >>  4) & 0xF);
                buf[7] = hexChar((value      ) & 0xF);
                buf[8] = '\r';
                buf[9] = '\n';
                buf[10] = 0;
                write(fd, buf, 10);

                gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
            }
        }
    }
}
#else
int main(void)
{
    uint32_t state, lastState = 0, pressed;

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    tsiInit(&tsiConfig);

    for (;;) {
        delay();

        state = tsiRead(&tsiConfig);

        pressed = state & ~lastState;

        if (pressed & BIT_5) {
            gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        }
        if (pressed & BIT_8) {
            gpioToggle(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        }
        if (pressed & BIT_7) {
            gpioToggle(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        }
        if (pressed & BIT_9) {
            gpioToggle(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        }

        lastState = state;
    }

    return 0;
}
#endif
