/*******************************************************************************
*
* demoCrc.c
*
* Shaun Weise
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

char testData[]        = "123456789";
char testAlignedData[] = "12345678";
uint32_t d = 0;

typedef enum {
    OUT_CRC16             = 0xBB3D,
    OUT_CRC16MODBUS       = 0x4B37,
    OUT_CRC16CITT_XMODEM  = 0x31C3,
    OUT_CRC16CITT         = 0x29B1,
    OUT_CRC32             = 0xCBF43926,
};

void fail(int line){
    static volatile int i;
    i = line;
}

int main(void)
{
    int crcFd;

    /* Install Standard I/O streams, then open &
     * configure UART3 */
    uart_install();
    int fd0 = fdevopen(stdin,  "uart3", 0, 0);
    int fd1 = fdevopen(stdout, "uart3", 0, 0);
    int fd2 = fdevopen(stderr, "uart3", 0, 0);
    assert(fd0 != -1);
    assert(fd1 != -1);
    assert(fd2 != -1);
    int uartFd = open("uart3", 0, 0);
    assert(uartFd != -1);
    ioctl(fd0, IO_IOCTL_UART_BAUD_SET, 9600);

    /* Install the CRC and open it */
    crc_install();
    crcFd = open("crc", 0, 0);
    assert(crcFd!=-1);

    /* CRC32 */
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_PRO_WIDTH, CRC_WIDTH_32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_ENABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC32);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 4);
    printf("\nTesting CRC32: %d==%d ? ",d,OUT_CRC32);
    if (d == OUT_CRC32) printf("Hell yeah!");
    else {
        printf("Try again?");
        fail(__LINE__);
    }

    /* .. for reset the crc module and start a new calc with this config
     * just set the seed again. */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC32);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 4);
    printf("\nTesting CRC32 again: %d==%d ? ",d,OUT_CRC32);
    if (d == OUT_CRC32) printf("Hell yeah! Woot!");
    else {
        printf("Wow, are you OK?");
        fail(__LINE__);
    }

    /* CRC32, but with 32bit writes*/
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_4BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_PRO_WIDTH, CRC_WIDTH_32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS_AND_BYTES); /*Fix f-ing endianess*/
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_ENABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC32);
    write(crcFd, (void *)testAlignedData,    strlen(testAlignedData));
    read(crcFd, &d, 4);
    printf("\nTesting CRC32 (32bin wide writes): %d==%d ? ",d,OUT_CRC32);
    if (d == OUT_CRC32) printf("Fast, no?");
    else {
        printf("You'll fix it later i guess");
        fail(__LINE__);
    }

    /* CRC16 */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_PRO_WIDTH, CRC_WIDTH_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 2);
    printf("\nTesting CRC16: %d==%d ? ",d,OUT_CRC16);
    if (d == OUT_CRC16) printf("Nice.");
    else {
        printf("!?#$");
        fail(__LINE__);
    }

    /* CRC16, but with 16bit writes */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_2BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_PRO_WIDTH, CRC_WIDTH_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS_AND_BYTES); /* Fix f-ing endianess */
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16);
    write(crcFd, (void *)testAlignedData,    strlen(testAlignedData));
    read(crcFd, &d, 2);
    printf("\nTesting CRC16 (16bit wide writes): %d==%d ? ",d,OUT_CRC16);
    if (d == OUT_CRC16) printf("Fair enough");
    else {
        printf("Not acceptable");
        fail(__LINE__);
    }

    /* CRC16CITT */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_PRO_WIDTH, CRC_WIDTH_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16CITT);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16CITT);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 2);
    printf("\nTesting CRC16CITT: %d==%d ? ",d,OUT_CRC16CITT);
    if (d == OUT_CRC16CITT) printf("Good.");
    else {
        printf("Bad.");
        fail(__LINE__);
    }

    /* CRC16CITT XMODEM */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_PRO_WIDTH, CRC_WIDTH_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16CITT);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16CITT_XMODEM);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 2);
    printf("\nTesting CRC16CITT_XMODEM: %d==%d ? ",d,OUT_CRC16CITT_XMODEM);
    if (d == OUT_CRC16CITT_XMODEM) printf("Sure...");
    else {
        printf("NO!!!!");
        fail(__LINE__);
    }


    /* Put everything away and go home */
    close(crcFd);
    close(uartFd);
    close(fd0);
    close(fd1);
    close(fd2);
    return 0;
}
