/*******************************************************************************
*
* adc.c
*
* jimmyMac!
*
* Low level driver for the Kinetis ADC module
*           ...because we live in an analog world
*
* See Ch 34 in K60P144M100SF2RM.pdf the TRM from Freescale
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
#include <string.h>
#include <errno.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"


#define ADC_BUFFER_SIZE 256
#define ADC_BUFFER_WRAP (ADC_BUFFER_SIZE - 1)

typedef struct {
    volatile uint32_t buffer[ADC_BUFFER_SIZE];
    volatile uint8_t  tailIdx;
    volatile uint8_t  headIdx;
    volatile uint8_t  length;
} adcBuffer_t;


typedef struct {
    int32_t            minor;
    volatile adc_t    *reg;
    volatile uint32_t *simScgcPtr;
    unsigned           simScgcEnBit;
    unsigned           simScgc5PortEn;
    int32_t            numSamples;
    int32_t          (*callBack)(uint32_t const *buf, int len);
    adcBuffer_t        adcBuffer;
#if 0
    unsigned             port;
    unsigned             sim;
    unsigned             simScgc5PortEn;
    uint32_t             txPin;
    uint32_t             rxPin;
    uint32_t             txPortCtrlBits;
    uint32_t             rxPortCtrlBits;
#endif
} adcDev_t;

typedef enum adcModule_e{
    ADC_MODULE_0,
    ADC_MODULE_1,
    NUM_ADC_MODULES,
} adcModule_t;



static adcDev_t adcList[NUM_ADC_MODULES] = {
    [ADC_MODULE_0] = {
        .minor          = ADC_MODULE_0,
        .reg            = ADC0_REG_PTR,
        .simScgcPtr     = SIM_SCGC6_PTR,
        .simScgcEnBit   = SIM_SCGC6_ADC0_ENABLE,
        .simScgc5PortEn = ADC0_PORT_ENABLE,
    },
    [ADC_MODULE_1] = {
        .minor          = ADC_MODULE_1,
        .reg            = ADC1_REG_PTR,
        .simScgcPtr     = SIM_SCGC3_PTR,
        .simScgcEnBit   = SIM_SCGC3_ADC1_ENABLE,
        .simScgc5PortEn = ADC1_PORT_ENABLE,

    },
};


static void adcNotify(adcDev_t *adc, adcBuffer_t *bufferPtr)
{
    uint32_t buf[bufferPtr->length];
    int i;
    for (i = 0; i < bufferPtr->length; i++) {
        buf[i] = bufferPtr->buffer[bufferPtr->headIdx];
        bufferPtr->headIdx = (bufferPtr->headIdx + 1)
            & ADC_BUFFER_WRAP;
    }
    adcList[adc->minor].callBack(buf, bufferPtr->length);
    bufferPtr->length = 0;
    return;
}
/******************************************************************************
* isrAdcX (void)
*
* Conversion complete ISR
*
******************************************************************************/
static void isrHandler(int minor)
{
    adcDev_t *adc = &adcList[minor];
    uint32_t data;
    adcBuffer_t *bufferPtr = &adcList[minor].adcBuffer;


    if (adc->reg->sc1a & ADC_SC1_COCO_BIT) {
        data = adc->reg->ra;
    }
    else if (adc->reg->sc1b & ADC_SC1_COCO_BIT) {
        data = adc->reg->rb;
    }
    else {
        /* Cant' happed ? */
        assert(0);
        return;
    }
    bufferPtr->buffer[bufferPtr->tailIdx] = data;
    bufferPtr->tailIdx = (bufferPtr->tailIdx + 1) & ADC_BUFFER_WRAP;
    bufferPtr->length++;

    if (adc->callBack && (bufferPtr->length >= adc->numSamples)) {
        adcNotify(adc, bufferPtr);
    }
    return;
}

static void isrAdc0(void)
{
    isrHandler(ADC_MODULE_0);
    return;
}

static void isrAdc1(void)
{
    isrHandler(ADC_MODULE_1);
    return;
}

/******************************************************************************
 *  calAdc
 *
 *  Performs ADC calibration per the TRM 34.4.7
 *
 *  returns TRUE is cal successful or FALSE if you are an a$$hole.
 ******************************************************************************/
static int calAdc (adcDev_t *adc)
{
    int retVal = TRUE;
    uint32_t value;
    uint32_t sc1aValue;
    uint32_t sc1bValue;

    value = adc->reg->sc3;
    value &= ~ADC_SC3_AVGS_MASK;
    value |= ADC_SC3_AVGE_BIT | ADC_SC3_AVGS_32;
    adc->reg->sc3 = value;

    value = adc->reg->cfg2;
    value &= ~ADC_CFG_ADLSTS_MASK;
    value |= ADC_CFG2_ADLSTS_ADCK_20;
    adc->reg->cfg2 = value;

    /* Turn off hw trigger */
    value = adc->reg->sc2;
    value &= ~ADC_SC2_ADTRG_BIT;
    adc->reg->sc2 = value;

    /* Turn off interrupts */
    sc1aValue = adc->reg->sc1a;
    adc->reg->sc1a &= ~ADC_SC1_AIEN_BIT;
    sc1bValue = adc->reg->sc1b;
    adc->reg->sc1b &= ~ADC_SC1_AIEN_BIT;


    /* Start cal */
    adc->reg->sc3 |= ADC_SC3_CAL_BIT;

    while (adc->reg->sc3 & ADC_SC3_CAL_BIT);

    if (adc->reg->sc3 & ADC_SC3_CALF_BIT) {
        retVal = FALSE;
    }
    else {
        uint32_t calVal = 0;

        calVal += adc->reg->clp0;
        calVal += adc->reg->clp1;
        calVal += adc->reg->clp2;
        calVal += adc->reg->clp3;
        calVal += adc->reg->clp4;
        calVal += adc->reg->clps;
        calVal /= 2; /* Shift etc if you think you are smart */
        calVal |= BIT_15;
        adc->reg->pg = calVal;

        calVal = 0;
        calVal += adc->reg->clm0;
        calVal += adc->reg->clm1;
        calVal += adc->reg->clm2;
        calVal += adc->reg->clm3;
        calVal += adc->reg->clm4;
        calVal += adc->reg->clms;
        calVal /= 2;
        calVal |= BIT_15;
        adc->reg->mg = calVal;

    }

    /* Restore interrupts */
    adc->reg->sc1a = sc1aValue;
    adc->reg->sc1b = sc1bValue;


    return retVal;
}
static int adcOpen(devoptab_t *dot)
{
    adcDev_t *adc;
    void  *isrPtr;
    uint32_t value;

    if (dot->priv) return FALSE; /* Device is already open */

    adc = &adcList[dot->min];
    dot->priv = adc;

#if 0
    /*
     * Config the SIM Clock Gate
     */

    SIM_SCGC5 |= adc->simScgc5PortEn;

    /*
     * Config the Port Controller
     */

#endif
    /*
     * Config the SIM Enable
     */
    *adc->simScgcPtr |= adc->simScgcEnBit;

    /* Initialization sequence described in the TRM SS 34.5.1.1 */

    /*
     * Calibrate ADC
     * See ss 34.4.7 in the TRM
     */
    while (!calAdc(adc));

    /*
     * Select input clock source CFG
     */

    /* NOTE: The analog clock (f_ADCK) must be between  1-18MHz
     * unless in the 16bit mode, when it must be between 2-12MHz.
     * (See K60P144M100SF.pdf pg 42).
     */



    /*  Unless you are running high sample rates, feel free to
     *  use the low power and keep a little ice at the poles...
     */
    value = ADC_CFG1_ADLPC_BIT;
                                               /* Divide the input clock by 1 */
    value |= (ADC_CFG1_ADIV_1 & ADC_CFG1_ADIV_MASK) << ADC_CFG1_ADIV_SHIFT;
                                               /* 12 Bit resolution */
    value |= (ADC_CFG1_MODE_12_BIT & ADC_CFG1_MODE_MASK) << ADC_CFG1_MODE_SHIFT;

                                                       /* Input the bus clock */
    value |= ADC_CFG1_ADICLK_BUS;
    adc->reg->cfg1 = value;

    /*
     * Select trigger source SC2
     */
    adc->reg->sc2 = 0; /* default the sonmonbitch */

    /*
     * Select conversions and averaging SC3
     */
                                                 /* Run continuous conversions */
    value = ADC_SC3_ADCO_BIT;
                                                 /* Use hardware averaging */
    value |= ADC_SC3_AVGE_BIT;

                        /* You know what they say, measure 4 times, cut once. */
    value |= ADC_SC3_AVGS_4;
    adc->reg->sc3 = value;

    /*
     * SC1n:
     * Select single or differential.
     * Set interrupts.
     * Select channel. SC1:SC1n
     */
    switch (adc->minor) {
    case ADC_MODULE_0:
        isrPtr = isrAdc0;
        break;
    case ADC_MODULE_1:
        isrPtr = isrAdc1;
        break;

    default:
        assert(0);
        return FALSE;
    }

    hwInstallISRHandler(ISR_ADC0 + adc->minor, isrPtr);

    value = ADC_SC1_ADCH_DISABLED;
    adc->reg->sc1a = value;
    adc->reg->sc1b = value;

    /*
     * Update PGA
     */
    /* Defaul value */


    return TRUE;
}

/*******************************************************************************
*
* This routine starts a DAC conversion
*
* RETURNS: Number conversions read
*
*******************************************************************************/
int32_t adcRead(devoptab_t *dot, const void *data, unsigned len)
{
    int32_t i;
    uint32_t *dataPtr = (uint32_t *) data;
    adcDev_t *adc;
    volatile uint32_t *adcSc1Ptr;
    volatile uint32_t *adcRPtr;
    uint32_t value;

    if (!dot || !dot->priv) return FALSE;
    else adc = (adcDev_t *) dot->priv;


    if (adc->reg->cfg2 & ADC_CFG2_MUXSEL_BIT) {
        adcSc1Ptr = &adc->reg->sc1b;
        adcRPtr  = &adc->reg->rb;
    }
    else {
        adcSc1Ptr = &adc->reg->sc1a;
        adcRPtr  = &adc->reg->ra;
    }

    for (i = 0; i < len; i++) {
        int readyRetry = 1000;
        value = adc->reg->sc1a;
        adc->reg->sc1a = value; /* SW trigger */

        while (!(*adcSc1Ptr & ADC_SC1_COCO_BIT) && --readyRetry) {
            value = *adcSc1Ptr;
        }

        if (readyRetry) {
            *dataPtr++ = *adcRPtr;
        }
        else {
            break;
        }
    }
    return dataPtr - (uint32_t *)data;
}

/*=============================================================================*/
/* POSIX FUNCTIONS                                                             */
/*=============================================================================*/

/*******************************************************************************/
/* adc_open_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'open' syscall:
 *      Check device name
 *      Create a device 'state' structure, hook it to the devoptab private ptr
 *      Enable the SIM SCGC for the device
 *      Initialize the device with a default configuration
 ********************************************************************************/
int adc_open_r (void *reent, devoptab_t *dot, int mode, int flags )
{
    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }

    /* Test the module instance */
    if ( dot->min >= NUM_ADC_MODULES ) {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open */
    if (adcOpen(dot)) {
        return TRUE;
    } else {
        /* Could not allocate memory */
        return FALSE;
    }


#if 0
    /* Determine the module instance */
    if (strcmp(DEVOPTAB_ADC0_STR, dot->name) == 0 ) {
        mod = ADC_MODULE_0;
    }
    else if (strcmp(DEVOPTAB_ADC1_STR, dot->name) == 0) {
        mod = ADC_MODULE_1;
    }
    else {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open if not already open */
    if (adcOpen(mod,dot)) {
        return TRUE;
    } else {
        /* Device is already open, is this an issue or not? */
        ((struct _reent *)reent)->_errno = EPERM;
        return FALSE;
    }
#endif
}

/*******************************************************************************/
/* adc_ioctl_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'ioctl' syscall:
 *      Implement any device specific commands.
 *          Commands are listed in hardware.h in the specific driver section
  *******************************************************************************/
int adc_ioctl(devoptab_t *dot, int cmd,  int flags)
{
    adcDev_t *adc;
    adcBuffer_t *bufferPtr;
    uint32_t value;
    int retVal = !ERROR;


    if (!dot || !dot->priv) return FALSE;
    else adc = (adcDev_t *) dot->priv;

    bufferPtr = &adcList[adc->minor].adcBuffer;

    switch (cmd) {
    case IO_IOCTL_ADC_CALL_BACK_SET:
        if (flags) {
            adcList[adc->minor].callBack = (void *) flags;
            /* Callback implies interupt driven */
            adc->reg->sc1a |= ADC_SC1_AIEN_BIT;
            adc->reg->sc1b |= ADC_SC1_AIEN_BIT;
        }
        break;
    case IO_IOCTL_ADC_CONVERSION_CONTINUOUS:
        if (flags) {
            adc->reg->sc3 |=  ADC_SC3_ADCO_BIT;
        }
        else {
            adc->reg->sc3 &= ~ADC_SC3_ADCO_BIT;
        }
        break;
    case IO_IOCTL_ADC_TRIGGER_SELECT:
        if (flags == IO_IOCTL_ADC_TRIGGER_SELECT_HW) {
            adc->reg->sc2 |=  ADC_SC2_ADTRG_BIT;
        }
        else {
            adc->reg->sc2 &= ~ADC_SC2_ADTRG_BIT;
        }
        break;
    case IO_IOCTL_ADC_CONVERSION_TIME_SELECT:
        if (flags == IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_SHORT_SAMPLE) {
            adc->reg->cfg1 &= ~ADC_CFG1_ADLSMP_BIT;
        }
        else {
            adc->reg->cfg1 |= ADC_CFG1_ADLSMP_BIT;
            adc->reg->cfg2 |= flags & ADC_CFG_ADLSTS_MASK;
        }
        break;
    case IO_IOCTL_ADC_CALIBRATE:
        retVal = calAdc(adc);
        break;
    case IO_IOCTL_ADC_AVERAGE_SELECT:
        value = adc->reg->sc3;
        switch (flags) {

        case ADC_SC3_AVGS_4:
            value |= (ADC_SC3_AVGE_BIT & ADC_SC3_AVGS_4);
            break;
        case ADC_SC3_AVGS_8:
            value |= (ADC_SC3_AVGE_BIT & ADC_SC3_AVGS_8);
            break;
        case ADC_SC3_AVGS_16:
            value |= (ADC_SC3_AVGE_BIT & ADC_SC3_AVGS_16);
            break;
        case ADC_SC3_AVGS_32:
            value |= (ADC_SC3_AVGE_BIT & ADC_SC3_AVGS_32);
            break;
        default:
            /* Send any other value to turn off averaging */
            value &= ~ADC_SC3_AVGE_BIT;
            break;
        }
        adc->reg->sc3 = value;
        break;
    case IO_IOCTL_ADC_RESOLUTION_SELECT:
        value = adc->reg->cfg1;
        value &= ~(ADC_CFG1_MODE_MASK << ADC_CFG1_MODE_SHIFT);
        value |= (flags & ADC_CFG1_MODE_MASK) << ADC_CFG1_MODE_SHIFT;
        adc->reg->cfg1 = value;
        break;
    case IO_IOCTL_ADC_CHANNEL_SELECT:
        if (flags & IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_B) {
            value = adc->reg->sc1b;
        }
        else {
            value = adc->reg->sc1a;
        }
        value &= ~IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK;
        value |= (flags & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK);
        if (flags & IO_IOCTL_ADC_CHANNEL_FLAGS_REGISER_B) {
            adc->reg->sc1b = value;
        }
        else {
            adc->reg->sc1a = value;
        }
        break;

    case IO_IOCTL_ADC_FLUSH_FIFO:
        {
            int32_t valueA, valueB;
            valueA = adc->reg->sc1a;
            valueB = adc->reg->sc1b;
            adc->reg->sc1a &= ~ADC_SC1_AIEN_BIT;
            adc->reg->sc1b &= ~ADC_SC1_AIEN_BIT;
            bufferPtr->tailIdx = 0;
            bufferPtr->length  = 0;
            adc->reg->sc1a = valueA;
            adc->reg->sc1b = valueB;
        }
        break;
    case IO_IOCTL_ADC_SAMPLE_SIZE_SET:
        adcList[adc->minor].numSamples = flags;
        break;
    default:
#if 0
            /*TODO Add cmds */
        IO_IOCTL_ADC_OFFSET_SET,
        IO_IOCTL_ADC_PGASET,
        IO_IOCTL_ADC_CLOCK_SELECT,
        IO_IOCTL_ADC_VREF_SELECT,
        IO_IOCTL_ADC_COMPARE_SELECT,
#endif

        assert(0);
        retVal = ERROR;
        break;
    }

    return retVal;
}

/*******************************************************************************/
/* adc_close_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'close' syscall:
 *      Disable the SIM SCGC for the device
 *      Free the device 'state' structure, unhook it to the devoptab private ptr
 *******************************************************************************/
int adc_close_r (void *reent, devoptab_t *dot )
{
    adcDev_t *adc = dot->priv;

    if (adc) {
        /* Disable the SIMSCGC for the adc module being used*/
        *adc->simScgcPtr &= ~adc->simScgcEnBit;
        /* Unhook the private adc structure */
        dot->priv = NULL;
        return TRUE;
    }
    else {
        return FALSE;
    }
}

/*******************************************************************************/
/* adc_read_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'read' syscall:
 *      Read data from the device
 *      Return the number of bytes read
 *******************************************************************************/
long adc_read_r (void *reent, devoptab_t *dot, void *buf, int len )
{
    /* You could just put your read function here, but I want switch between
     * polled & interupt functions here at a later point.*/
    return adcRead(dot, buf, len);
}

int adc_install(void)
{
    int ret = TRUE;

    if( !deviceInstall(DEV_MAJ_ADC, adc_open_r, adc_ioctl, adc_close_r,
                                                           NULL, adc_read_r) ){
        ret = FALSE;
    }
    if( !deviceRegister("adc0", DEV_MAJ_ADC, ADC_MODULE_0,  NULL) ) {
        ret =  FALSE;
    }
    if( !deviceRegister("adc1", DEV_MAJ_ADC, ADC_MODULE_1,  NULL) ) {
        ret =  FALSE;
    }

    return ret;
}


