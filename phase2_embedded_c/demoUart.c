/*******************************************************************************
*
* demoUart.c
*
* James McAnanama
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"
#include "util.h"

enum {
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
};

#define COLOUR_STRINGS "ORANGE\t", "YELLOW\t", "GREEN\t", "BLUE\n\v\r"

enum {
    UPDATE_NONE,
    UPDATE_HELP,
    UPDATE_QUIT,
};

static int updateFlags;
static int fd;

/******************************************************************************
* isr_uart3_status_sources(void)
*
* Status ISR definition.
*
******************************************************************************/
static void isrUartStatusSources(void)

{
    char readString[256] = {'\0'};
    int32_t len;
    len = read(fd, (uint8_t *)readString, 1);
    if (len) {
        if (readString[0] == 'Q' || readString[0] == 'q') {
            updateFlags |= UPDATE_QUIT;
        }
        else {
            updateFlags |= UPDATE_HELP;
        }
    }

   return;
}


int main(void)
{
    static char *colourStrings[] = { COLOUR_STRINGS };

    /* Install uart into the device table before using it */
    uart_install();



    fd = open("uart3", 0, 0);
    if (fd==-1) {
        assert(0);
    }


    /* This feels dirty, but I like it... Let me know if I am sinning here.
     * I want to register an RX interupt handler here in this file. */
    ioctl(fd, IO_IOCTL_UART_ENABLE_RX_INTERUPT, (int)isrUartStatusSources);

   /* TODO Shaun printf is broken w the uart_install() changeds. */
   printf("The start of something good...\r\n"); /* StdOut is uart3 */

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        write(fd, colourStrings[ORANGE], strlen(colourStrings[ORANGE]));
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        write(fd, colourStrings[YELLOW], strlen(colourStrings[YELLOW]));
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        write(fd, colourStrings[GREEN], strlen(colourStrings[GREEN]));
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        write(fd, colourStrings[BLUE], strlen(colourStrings[BLUE]));

        delay();
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        delay();
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        delay();
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

        if (updateFlags & UPDATE_HELP) {
            write(fd, "\r\n Press Q to quit \r\n",
               strlen("\r\n Press Q to quit \r\n"));
            updateFlags &= ~UPDATE_HELP;
        }


        if (updateFlags & UPDATE_QUIT) {
            write(fd, "\r\n BuBye \r\n \r\n",
                    strlen("\r\n BuBye \r\n \r\n"));
            //printf("\r\n");
            //printf("BuBye \r\n");
            //printf("==================================== \r\n");
            //printf("\r\n");
            updateFlags &= ~UPDATE_QUIT;
            break;
        }

    }


    gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
    gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);


    for (;;) {
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        printf("Shaun.\n\n"); /* Shaun likes seeing his name. */

        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
    }

    close(fd);
    return 0;
}
