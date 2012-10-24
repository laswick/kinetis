/*******************************************************************************
*
* demoSpi.c
*
* Shaun Weise
*
* This demo demonstrates basic SPI functionality. SPI2 is connected to a
* digilent PmodCls, a 16x2 lcd character display. It features SPI, I2C and UART
* interfaces so its perfect for testing out these interfaces on new platforms.
* Write character data to it and it will display the characters, no
* initialization required. There are special control sequences to control the
* cursor, current line, etc.
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

/* My name, in a character array. I like seeing my name. */
char shaun[]  = { 'S','h','a','u','n' };
char buff[10] = { 0 };

/* PmodCls Serial Commands */
char scrCmdClear[]      = {0x1B, '[','0','j' };
char scrCmdReset[]      = {0x1B, '[','0','*' };
char scrCmdDispMode16[] = {0x1B, '[','0','h' };
char scrCmdDispMode40[] = {0x1B, '[','1','h' };
char scrCmdGotoLine1[]  = {0x1B, '[','0',';','0','0','H' };
char scrCmdGotoLine2[]  = {0x1B, '[','1',';','0','0','H' };
char scrCmdScrollL1[]   = {0x1B, '[','0','1','@' };
char scrCmdScrollR1[]   = {0x1B, '[','0','1','A' };
char scrCmdDispEnBklghtOn[]  = {0x1B, '[','3','e' }; /* Screwed up */
char methodStrings[NUM_SPI_METHODS][20] = { "Polled", "Interrupts", "DMA" };


static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

int main(void)
{
    int spiFd0,spiFd1,spiFd2;
    int uartFd;
    int i;
    int tests[NUM_SPI_METHODS] = { FALSE };

    /* Configure the clock for the uarts */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

    /* Install Standard I/O streams, then open &
     * configure UART3 */
    uart_install();
    if( fdevopen(stdin,  "uart3", 0, 0) == -1) assert(0);
    if( fdevopen(stdout, "uart3", 0, 0) == -1) assert(0);
    if( fdevopen(stderr, "uart3", 0, 0) == -1) assert(0);
    uartFd = open("uart3", 0, 0);
    assert(uartFd != -1);
    ioctl(uartFd, IO_IOCTL_UART_BAUD_SET, 115200);
    printf("\r\n");
    printf("\r\n = SPI TEST APPLICATION = \r\n");

    /* Install spi into the device table before using it */
    spi_install();
    spiFd0 = open("spi0", 0, 0);
    assert(spiFd0 != -1);
#if 0
    spiFd1 = open("spi1", 0, 0);
    assert(spiFd1 != -1);
    spiFd0 = open("spi2", 0, 0);
    assert(spiFd0 != -1);
#endif

    ioctl(spiFd0, IO_IOCTL_SPI_SET_PORT_PCRS, 0);
    ioctl(spiFd0, IO_IOCTL_SPI_SET_BAUD, SPI_BAUDRATE_CLKDIV_1024);
    ioctl(spiFd0, IO_IOCTL_SPI_SET_SCLK_MODE, SPI_SCLK_MODE_0);
    ioctl(spiFd0, IO_IOCTL_SPI_SET_FMSZ, 8);
    ioctl(spiFd0, IO_IOCTL_SPI_SET_OPTS, SPI_OPTS_MASTER);
    ioctl(spiFd0, IO_IOCTL_SPI_SET_CS, SPI_CS_0);
    ioctl(spiFd0, IO_IOCTL_SPI_SET_CS_INACT_STATE, SPI_CS_0_INACT_HIGH);

    /* Test out all the differnt methods */
    for(i = SPI_METHOD_POLLED; i < NUM_SPI_METHODS; i++) {
        ioctl(spiFd0, IO_IOCTL_SPI_SET_METHOD, i);
        write(spiFd0, shaun, sizeof(shaun));
        delay();
        delay();
        read(spiFd0, buff, sizeof(shaun));
        if (memcmp((void*)shaun, (void*)buff, sizeof(shaun)) == 0)
            tests[i] = TRUE;
        memset(buff, 0, sizeof(buff));
    }

    for(i = SPI_METHOD_POLLED; i < NUM_SPI_METHODS; i++) {
        printf("SPI Test: %13s:", methodStrings[i]);
        if (tests[i])
            printf("PASS\r\n");
        else
            printf("FAIL\r\n");
    }

    while(1){
    }

    return 0;
}
