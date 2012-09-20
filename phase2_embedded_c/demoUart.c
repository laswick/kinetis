/*******************************************************************************
*
* demoUart.c
*
* Simple uart demo.
* This demo supports the following serial interface
*
* Byte 0: Framing Byte
*       0x41, 'A' Valid frame
* Byte 1: Command Byte
*       0x30, '0' Do nothing
*       0x31, '1' Echo help message
*       0x32, '2' Toggle LED status
*       0x32, '3' Quit
* Byte 2: Select language for echo
*       0x30, '0' English
*       0x31, '1' Drunken English
* Byte 3: CRC
*       XOR Bytes 0-2.
*       e.g. A,1,0 would be 0x41 ^ 0x31 ^ 0x30 = 0x40 ('@')
*       e.g. A,2,0 would be 0x41 ^ 0x32 ^ 0x30 = 0x43 ('C')
*       e.g. A,3,0 would be 0x41 ^ 0x33 ^ 0x30 = 0x42 ('B')
*
*
*
*
* James McAnanama
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
* Canada Day 2012
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

#define COLOUR_STRINGS "ORANGE\t", "YELLOW\t", "GREEN\t", "BLUE\r\n"
enum {
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
};

enum {
    UPDATE_NONE,
    UPDATE_CMD,
    UPDATE_GARBAGE,
};

static int updateFlags;
static int fd;

enum {
    SERIAL_CMD_NOTHING    = '0',
    SERIAL_CMD_HELP       = '1',
    SERIAL_CMD_TOGGLE_LED = '2',
    SERIAL_CMD_QUIT       = '3',
};

enum {
    HELP_ENGLISH = '0',
    HELP_DRUNK   = '1',
};

typedef struct {
    int32_t cmd;
    int32_t style;
    char *helpStringEng;
    char *helpStringDEng;
    char garbage[256];
} msg_t;

static msg_t msg = {
    .cmd           = SERIAL_CMD_HELP,
    .style         = HELP_ENGLISH,
    .helpStringEng =     "\r\n"
                         "Serial Interface: \r\n"
                         "Byte 0: Framing byte 'A', \r\n"
                         "Byte 1: Cmd: \r\n"
                         "          '1' Echo this help message, \r\n"
                         "          '2' Toggle LED flashing. \r\n"
                         "          '3' Quit. \r\n"
                         "Byte 2: Style: '0' English, '1' Drunken English \r\n"
                         "Byte 3: CRC: XOR Bytes 0 - 2 \r\n"
                         "Terminate with a carriage return \r\n"
                         "Try these: \r\n"
                         "A10@, A11A, A20C, A30B \r\n",

    .helpStringDEng =    "\r\n"
                         "Whaaaa? ... \r\n"
                         "I you say 'A' Eh?  HAAAAA! \r\n"
                         "FnuI'll go.   .?  \r\n"
                         "What?  \r\n"
                         "I love you man... ZZZZzzzzz \r\n",
};

/******************************************************************************
* uartCallBackHandler
*
* Parse RX messages.
*
******************************************************************************/
static void uartCallBackHandler(uint8_t const *buf, int len)
{
    if (len >= 4 && buf[0] == 'A') {
        int i;
        int crc = 0;
        msg.cmd = SERIAL_CMD_NOTHING;
        for (i = 0; i < 3; i++) {
            crc ^= buf[i];
        }
        if (crc == buf[3]) {
            msg.cmd   = buf[1];
            msg.style = buf[2];
            updateFlags |= UPDATE_CMD;
        }
    }
    else {
        len = len > 255 ? 255 : len;
        memcpy(msg.garbage, buf, len);
        msg.garbage[len] = '\0';
        updateFlags |= UPDATE_GARBAGE;
    }

   return;
}


int main(void)
{
    static char *colourStrings[] = { COLOUR_STRINGS };
    int quit  = FALSE;
    int blink = FALSE;

    updateFlags = UPDATE_CMD; /* Launch w help msg */


    /* Install uart into the device table before using it */
    uart_install();

    fd = open("uart3", 0, 0); /* STDIN */
    if (fd != 0) {
        assert(0);
    }
    fd = open("uart3", 0, 0); /* STDOUT */
    if (fd != 1) {
        assert(0);
    }
    fd = open("uart3", 0, 0); /* STDERR */
    if (fd != 2) {
        assert(0);
    }

    fd = open("uart3", 0, 0);
    if (fd==-1) {
        assert(0);
    }


    /*
     * Casting the callback down the (int) flag shoot feels dirty,
     * but the leprechaun that screams in my ear told me to do it...
     */
    ioctl(fd, IO_IOCTL_UART_CALL_BACK_SET, (int)uartCallBackHandler);

                                          /* Call me when the user hits enter */
    ioctl(fd, IO_IOCTL_UART_TERMINATOR_SET, '\r');
    ioctl(fd, IO_IOCTL_UART_BAUD_SET, 115200);

    /* TODO Shaun printf is broken w the uart_install() changeds. */
    printf("The start of something good...\r\n"); /* StdOut is uart3 */

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);

    while (!quit) {
        if (blink) {
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
        }


        if (updateFlags & UPDATE_CMD) {
            updateFlags &= ~UPDATE_CMD;
            switch (msg.cmd) {
            case SERIAL_CMD_HELP:
                if (msg.style == HELP_ENGLISH) {
                    write(fd, msg.helpStringEng, strlen(msg.helpStringEng));
                }
                else {
                    write(fd, msg.helpStringDEng, strlen(msg.helpStringDEng));
                }
                break;
            case SERIAL_CMD_QUIT:
                write(fd, "\r\n BuBye \r\n \r\n",
                   strlen("\r\n BuBye \r\n \r\n"));
                quit = TRUE;
                break;
            case SERIAL_CMD_TOGGLE_LED:
                blink = !blink;
                break;
            }
        }

        if (updateFlags & UPDATE_GARBAGE) {
            updateFlags &= ~UPDATE_GARBAGE;
            write(fd, "\r\n GARGAGE: ",
                strlen("\r\n GARBAGE:"));
            write(fd, msg.garbage, strlen(msg.garbage));
            write(fd, "\r\n",
                strlen("\r\n"));

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
