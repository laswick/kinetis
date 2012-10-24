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
#include "util.h"

char testData[]        = "123456789";
char testAlignedData[] = "12345678";
uint32_t d = 0;

typedef enum {
    OUT_CRC16             = 0xBB3D,
    OUT_CRC16_ALIGN       = 0x3C9D,
    OUT_CRC16MODBUS       = 0x4B37,
    OUT_CRC16CITT_XMODEM  = 0x31C3,
    OUT_CRC16CITT         = 0x29B1,
    OUT_CRC16CITT_ALIGN   = 0xA12B,
    OUT_CRC32             = 0xCBF43926,
    OUT_CRC32_ALIGN       = 0x9AE0DAAF,
};

void fail(int line){
    static volatile int i;
    i = line;
}

int main(void)
{
    int crcFd;

    /* Configure the clock for the uarts */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

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
    ioctl(uartFd, IO_IOCTL_UART_BAUD_SET, 115200);
    printf("\r\n");
    printf("\r\n = CRC TEST APPLICATION = \r\n");

    /* Install the CRC and open it */
    crc_install();
    crcFd = open("crc", 0, 0);
    assert(crcFd!=-1);

    /* CRC32 */
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TYPE,      CRC_TYPE_32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS_IN_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_ENABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC32);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 4);
    printf("\r\nTesting CRC32           : 0x%8x==0x%8x ? ",d,OUT_CRC32);
    if (d == OUT_CRC32) printf("OK! : Hell yeah!");
    else {
        printf("WTF? : Try again?");
        fail(__LINE__);
    }

    /* .. for reset the crc module and start a new calc with this config
     * just set the seed again. */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC32);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 4);
    printf("\r\nTesting CRC32           : 0x%8x==0x%8x ? ",d,OUT_CRC32);
    if (d == OUT_CRC32) printf("OK! : Hell yeah! Woot!");
    else {
        printf("WTF? : Wow, are you OK?");
        fail(__LINE__);
    }

    /* CRC32, but with 32bit writes*/
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_4BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TYPE,      CRC_TYPE_32);
    /* Need to also transpose the bytes here because of the endians */
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_ENABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC32);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC32);
    write(crcFd, (void *)testAlignedData,    strlen(testAlignedData));
    read(crcFd, &d, 4);
    printf("\r\nTesting CRC32 (32bit)   : 0x%8x==0x%8x ? ",d,OUT_CRC32_ALIGN);
    if (d == OUT_CRC32_ALIGN) printf("OK! : Fast, no?");
    else {
        printf("WTF? : You'll fix it later i guess");
        fail(__LINE__);
    }

    /* CRC16 */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TYPE,      CRC_TYPE_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS_IN_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 2);
    printf("\r\nTesting CRC16           : 0x%8x==0x%8x ? ",d,OUT_CRC16);
    if ((uint16_t)d == OUT_CRC16) printf("OK! : Nice.");
    else {
        printf("WTF? : !?#$");
        fail(__LINE__);
    }

    /* CRC16, but with 16bit writes */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_2BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TYPE,      CRC_TYPE_16);
    /* !WTF! - I have to add bits AND BYTES transposition here! */
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_BITS_AND_BYTES);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16);
    write(crcFd, (void *)testAlignedData,    strlen(testAlignedData));
    read(crcFd, &d, 2);
    printf("\r\nTesting CRC16 (16bit)   : 0x%8x==0x%8x ? ",d,OUT_CRC16_ALIGN);
    if ((uint16_t)d == OUT_CRC16_ALIGN) printf("OK! : Fair enough");
    else {
        printf("WTF? : Not acceptable");
        fail(__LINE__);
    }

    /* CRC16CITT */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TYPE,      CRC_TYPE_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16CITT);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16CITT);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 2);
    printf("\r\nTesting CRC16CITT       : 0x%8x==0x%8x ? ",d,OUT_CRC16CITT);
    if ((uint16_t)d == OUT_CRC16CITT) printf("OK! : Good.");
    else {
        printf("WTF? : Bad.");
        fail(__LINE__);
    }

    /* CRC16CITT XMODEM */
    d = 0;
    ioctl(crcFd, IO_IOCTL_CRC_SET_DWW,       CRC_DWW_BYTE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TYPE,      CRC_TYPE_16);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOT,       CRC_TOT_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_TOTR,      CRC_TOTR_NONE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_FXOR,      CRC_FXOR_DISABLE);
    ioctl(crcFd, IO_IOCTL_CRC_SET_POLY,      CRC_POLY_CRC16CITT);
    ioctl(crcFd, IO_IOCTL_CRC_SET_SEED,      CRC_SEED_CRC16CITT_XMODEM);
    write(crcFd, (void *)testData, strlen(testData));
    read(crcFd, &d, 2);
    printf("\r\nTesting CRC16CITT_XMODEM: 0x%8x==0x%8x ? ",d,OUT_CRC16CITT_XMODEM);
    if ((uint16_t)d == OUT_CRC16CITT_XMODEM) printf("OK! : Sure...");
    else {
        printf("WTF? : NO!!!!");
        fail(__LINE__);
    }

    printf("\r\n");

    while (1) {
        /* 'while (1)' kills computers */
    }

    /* Put everything away and go home */
    close(crcFd);
    close(uartFd);
    close(fd0);
    close(fd1);
    close(fd2);
    return 0;
}
