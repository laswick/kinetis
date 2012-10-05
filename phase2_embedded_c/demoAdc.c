/*******************************************************************************
*
* demoAdc.c
*
* Simple adc demo.
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


static int updateFlags;
static int fdUart;
static int fdAdc;

#define MAX_SAMPLES 256
typedef struct {
    char    *helpString;
    int      len;
    uint32_t data[MAX_SAMPLES];
} msg_t;

static msg_t msg = {
    .helpString =     "\r\n"
                         "Turn the pot... \r\n"
                         "\r\n",
};

enum {
    UPDATE_DATA = 1 << 0,
};

/******************************************************************************
* adcCallBackHandler
*
* Parse RX messages.
*
******************************************************************************/
static void adcCallBackHandler(uint32_t const *buf, int len)
{
    len = len > MAX_SAMPLES ? MAX_SAMPLES : len;
    if (len) {
        memcpy(msg.data, buf, len * 4);
        msg.len = len;
        updateFlags |= UPDATE_DATA;

    }
   return;
}

static void hex2HexStr(char *string, uint32_t value, int addLFCR)
{
    int i;
    int k = 0;

    string[k++] = '0';
    string[k++] = 'x';
    for (i = 3; i >= 0; i--) {
        char charVal = (value >> 4 * i) & 0xf;
        if (charVal < 0xa){
            string[k++] = '0' + charVal;
        }
        else {
            switch (charVal) {
                case 0xa:
                    string[k++] = 'a';
                    break;
                case 0xb:
                    string[k++] = 'b';
                    break;
                case 0xc:
                    string[k++] = 'c';
                    break;
                case 0xd:
                    string[k++] = 'd';
                    break;
                case 0xe:
                    string[k++] = 'e';
                    break;
                case 0xf:
                    string[k++] = 'f';
                    break;
            }
        }
    }
    if (addLFCR) {
        string[k++] = '\r';
        string[k++] = '\n';
    }
    string[k++] = '\0';
    return;
}

static void setClock(void)
{
    /* -------- 100 MHz (external clock) -----------
     * Configure the Multipurpose Clock Generator output to use the external
     * clock locked with a PLL at the maximum frequency of 100MHZ
     *
     * For PLL, the dividers must be set first.
     *
     * System:  100 MHz
     * Bus:      50 MHz
     * Flexbus:  50 MHz
     * Flash:    25 MHz
     */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

    return;


}


int main(void)
{
    int blink = TRUE;
    int quit  = FALSE;


    setClock();

    /* Install uart and adc into the device table before using them */
    uart_install();
    adc_install();

    fdUart = open("uart3", 0, 0);
    if (fdUart==-1) {
        assert(0);
    }
    ioctl(fdUart, IO_IOCTL_UART_BAUD_SET, 115200);

    fdAdc = open("adc1", 0, 0);
    if (fdAdc==-1) {
        assert(0);
    }

    ioctl(fdAdc, IO_IOCTL_ADC_CALIBRATE, TRUE);


    ioctl(fdAdc, IO_IOCTL_ADC_SAMPLE_SIZE_SET, 10);
    ioctl(fdAdc, IO_IOCTL_ADC_CALL_BACK_SET, (int)adcCallBackHandler);
    ioctl(fdAdc, IO_IOCTL_ADC_TRIGGER_SELECT, IO_IOCTL_ADC_TRIGGER_SELECT_SW);
    ioctl(fdAdc, IO_IOCTL_ADC_CONVERSION_CONTINUOUS, TRUE);
    ioctl(fdAdc, IO_IOCTL_ADC_CONVERSION_TIME_SELECT,
                 IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_20);
    ioctl(fdAdc, IO_IOCTL_ADC_AVERAGE_SELECT, IO_IOCTL_ADC_FLAGS_AVGS_4);
    ioctl(fdAdc, IO_IOCTL_ADC_RESOLUTION_SELECT, IO_IOCTL_ADC_RES_FLAGS_12_BIT);
    ioctl(fdAdc, IO_IOCTL_ADC_CLOCK_SELECT, IO_IOCTL_ADC_FLAGS_ADICLK_BUS);

    ioctl(fdAdc, IO_IOCTL_ADC_CHANNEL_SELECT,
                 (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_A
                    | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DM1
                       & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));

    //printf("The start of something good...\r\n"); /* StdOut is uart3 */

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);

    while (!quit) {
        if (blink) {
            delay();
            gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
            delay();
            gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
        }


        if (updateFlags & UPDATE_DATA) {
            char string[10];
            int i;

            updateFlags &= ~UPDATE_DATA;
            write(fdUart, "DATA:", strlen("DATA:"));
            write(fdUart, "\r\n",
                    strlen("\r\n"));
            for (i = 0; i < msg.len; i++) {
                /* Vararg not working? printf(">%x \r\n", msg.data[i]); */
                hex2HexStr(string, msg.data[i], TRUE);
                write(fdUart, string, strlen(string));
            }
        }


    }
    close(fdUart);
    close(fdAdc);
    return 0;
}
