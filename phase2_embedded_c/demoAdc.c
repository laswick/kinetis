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
static int fdPot;
static int fdThrmCpl;

#define MAX_SAMPLES 256
typedef struct {
    char    *helpString;
    int      len;
    uint32_t data[MAX_SAMPLES];
} msg_t;

static msg_t potMsg = {
    .helpString =     "\r\n"
                         "Turn the pot... \r\n"
                         "\r\n",
};

enum {
    UPDATE_POT_DATA = 1 << 0,
    UPDATE_TC_DATA  = 1 << 1,
};

/******************************************************************************
* potCallBackHandler
*
* Parse RX messages.
*
******************************************************************************/
static void potCallBackHandler(uint32_t const *buf, int len)
{
    len = len > MAX_SAMPLES ? MAX_SAMPLES : len;
    if (len) {
        memcpy(potMsg.data, buf, len * 4);
        potMsg.len = len;
        updateFlags |= UPDATE_POT_DATA;

    }
   return;
}

static msg_t tcMsg = {
    .helpString =     "\r\n"
                         "Aim the thermocouple... \r\n"
                         "\r\n",
};

/******************************************************************************
* tcCallBackHandler
*
* Parse RX messages.
*
******************************************************************************/
static void tcCallBackHandler(uint32_t const *buf, int len)
{
    len = len > MAX_SAMPLES ? MAX_SAMPLES : len;
    if (len) {
        memcpy(tcMsg.data, buf, len * 4);
        tcMsg.len = len;
        updateFlags |= UPDATE_TC_DATA;

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
    clockSetDividers(DIVIDE_BY_4, DIVIDE_BY_4, DIVIDE_BY_4, DIVIDE_BY_4);
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

    fdPot = open("adc1", 0, 0);
    if (fdPot==-1) {
        assert(0);
    }

    ioctl(fdPot, IO_IOCTL_ADC_CALIBRATE, TRUE);


    ioctl(fdPot, IO_IOCTL_ADC_SAMPLE_SIZE_SET, 10);
    ioctl(fdPot, IO_IOCTL_ADC_CALL_BACK_SET, (int)potCallBackHandler);
    ioctl(fdPot, IO_IOCTL_ADC_TRIGGER_SELECT, IO_IOCTL_ADC_TRIGGER_SELECT_SW);
    ioctl(fdPot, IO_IOCTL_ADC_CONVERSION_CONTINUOUS, TRUE);
    ioctl(fdPot, IO_IOCTL_ADC_CONVERSION_TIME_SELECT,
                 IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_20);
    ioctl(fdPot, IO_IOCTL_ADC_AVERAGE_SELECT, IO_IOCTL_ADC_FLAGS_AVGS_4);
    ioctl(fdPot, IO_IOCTL_ADC_RESOLUTION_SELECT, IO_IOCTL_ADC_RES_FLAGS_12_BIT);
    ioctl(fdPot, IO_IOCTL_ADC_CLOCK_SELECT, IO_IOCTL_ADC_FLAGS_ADICLK_BUS);
    ioctl(fdPot, IO_IOCTL_ADC_DIFFERENTIAL_SET,
                (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_A
                  | IO_IOCTL_ADC_DIFF_FLAGS_SINGLE_ENDED));

    ioctl(fdPot, IO_IOCTL_ADC_COMPARE_HIGH_LOW_SET,
                 IO_IOCTL_ADC_COMPARE_HIGH_LOW_FLAG_GREATER);

    ioctl(fdPot, IO_IOCTL_ADC_CHANNEL_SELECT,
                 (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_A
                    | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DM1
                       & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));




    fdThrmCpl = open("adc0", 0, 0);
    if (fdThrmCpl ==-1) {
        assert(0);
    }

    ioctl(fdThrmCpl, IO_IOCTL_ADC_RESOLUTION_SELECT,
                     IO_IOCTL_ADC_RES_FLAGS_12_BIT);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_VREF_SELECT, IO_IOCTL_ADC_VREF_FLAGS_ALT);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_PGASET, IO_IOCTL_ADC_PGA_FLAGS_GAIN_4);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_CALIBRATE, TRUE);

    ioctl(fdThrmCpl, IO_IOCTL_ADC_SAMPLE_SIZE_SET, 10);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_CALL_BACK_SET, (int)tcCallBackHandler);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_TRIGGER_SELECT,
                     IO_IOCTL_ADC_TRIGGER_SELECT_SW);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_CONVERSION_CONTINUOUS, TRUE);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_CONVERSION_TIME_SELECT,
                 IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_20);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_AVERAGE_SELECT, IO_IOCTL_ADC_FLAGS_AVGS_32);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_CLOCK_SELECT, IO_IOCTL_ADC_FLAGS_ADICLK_BUS);
    ioctl(fdThrmCpl, IO_IOCTL_ADC_DIFFERENTIAL_SET,
                    (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_A
                      | IO_IOCTL_ADC_DIFF_FLAGS_DIFFERENTIAL));


    ioctl(fdThrmCpl, IO_IOCTL_ADC_CHANNEL_SELECT,
                     (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_A
//                        | (IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DP0
                          | (IO_IOCTL_ADC0_CHANNEL_FLAGS_PGA0_DP1
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


        if (updateFlags & UPDATE_POT_DATA) {
            char string[10];
            static int32_t compareVal;
            static int32_t useGT = TRUE;
            int i;

            updateFlags &= ~UPDATE_POT_DATA;
            write(fdUart, "DATA:", strlen("DATA:"));
            write(fdUart, "\r\n",
                    strlen("\r\n"));
            for (i = 0; i < potMsg.len; i++) {
                /* Vararg not working? printf(">%x \r\n", potMsg.data[i]); */
                hex2HexStr(string, potMsg.data[i], TRUE);
                write(fdUart, string, strlen(string));

                compareVal = potMsg.data[i];

                ioctl(fdPot, IO_IOCTL_ADC_COMPARE_VALUES_SET,
                        compareVal & IO_IOCTL_ADC_COMPARE_VAL_MASK);

                if (compareVal >= 0xff0 && useGT) {
                    useGT = FALSE;
                    ioctl(fdPot, IO_IOCTL_ADC_COMPARE_HIGH_LOW_SET,
                            IO_IOCTL_ADC_COMPARE_HIGH_LOW_FLAG_LESS);
                }
                else if (compareVal <= 5 && !useGT){
                    useGT = TRUE;
                    ioctl(fdPot, IO_IOCTL_ADC_COMPARE_HIGH_LOW_SET,
                            IO_IOCTL_ADC_COMPARE_HIGH_LOW_FLAG_GREATER);
                }
                ioctl(fdPot, IO_IOCTL_ADC_COMPARE_ENABLE, TRUE);
                ioctl(fdPot, IO_IOCTL_ADC_COMPARE_RANGE_SET, FALSE);
            }
            /* Set the channel again trigger - needed because changing the
             * compare setpoint breaks the continous conversion mode */
            ioctl(fdPot, IO_IOCTL_ADC_CHANNEL_SELECT,
                 (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_A
                    | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DM1
                       & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));


        }

        if (updateFlags & UPDATE_TC_DATA) {
            char string[10];
            int i;

            updateFlags &= ~UPDATE_TC_DATA;
            write(fdUart, "\tTEMP:", strlen("\tTEMP:"));
            write(fdUart, "\r\n",
                    strlen("\r\n"));
            for (i = 0; i < tcMsg.len; i++) {
                /* Vararg not working? printf("\t>%x \r\n", tcMsg.data[i]); */
                string[0] = '\t';
                //hex2TempStr(&string[1], tcMsg.data[i], TRUE);
                hex2HexStr(&string[1], tcMsg.data[i], TRUE);
                write(fdUart, string, strlen(string));
            }
        }



    }
    close(fdUart);
    close(fdPot);
    return 0;
}
