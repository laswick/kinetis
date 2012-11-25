/*******************************************************************************
*
* ftm.c
*
* jimmyMac!
*
* Low level driver for the Kinetis FTM module
*           ...because we all should be more flexible
*
* See Ch 39 in K60P144M100SF2RM.pdf the TRM from Freescale
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"


#define FTM0_PORT PORTC
#define FTM0_PWM1_PIN 1
#define FTM0_PWM2_PIN 2
#define FTM0_PWM3_PIN 3
#define FTM0_PWM4_PIN 4
#define FTM0_PWM_MUX  (PORT_MUX_ALT4)

#define FTM1_PORT PORTB
#define FTM1_QD_PHA_PIN 0
#define FTM1_QD_PHB_PIN 1
#define FTM1_MUX_QD (PORT_MUX_ALT6)

#define FTM2_PORT PORTB
#define FTM2_QD_PHA_PIN 18
#define FTM2_QD_PHB_PIN 19
#define FTM2_MUX_QD (PORT_MUX_ALT6)

typedef struct {
    volatile ftm_t *ftm;
    volatile uint32_t *simScgcPtr;
    unsigned           simScgcEnBit;
    unsigned pinQDPhA;
    unsigned pinQDPhB;
    unsigned pinQDMux;
    unsigned pinCh[4];
    unsigned pinChMux;
    unsigned port;
    void (*isr)(void);
    int  ftmMode;
    void (*callBack)(int32_t index);
} ftmCtrl_t;

static void isrFtm0(void);
static void isrFtm1(void);
static void isrFtm2(void);

static ftmCtrl_t ftmCtrl[MAX_FTM] = {
    [FTM_0] = {
        .ftm = ((volatile ftm_t * const) FTM0_CTRL_BASE),
        .simScgcPtr   = SIM_SCGC6_PTR,
        .simScgcEnBit = SIM_SCGC6_FTM0_ENABLE,
        .port         = FTM0_PORT,
        .pinCh        = { FTM0_PWM1_PIN, FTM0_PWM2_PIN,
                          FTM0_PWM3_PIN, FTM0_PWM4_PIN},
        .pinChMux     = FTM0_PWM_MUX,
        .isr          = isrFtm0,
    },
    [FTM_1] = {
        .ftm = ((volatile ftm_t * const) FTM1_CTRL_BASE),
        .simScgcPtr   = SIM_SCGC6_PTR,
        .simScgcEnBit = SIM_SCGC6_FTM1_ENABLE,
        .port         = FTM1_PORT,
        .pinQDPhA     = FTM1_QD_PHA_PIN,
        .pinQDPhB     = FTM1_QD_PHB_PIN,
        .pinQDMux     = FTM1_MUX_QD,
        .isr          = isrFtm1,
    },
    [FTM_2] = {
        .ftm = ((volatile ftm_t * const) FTM2_CTRL_BASE),
        .simScgcPtr   = SIM_SCGC3_PTR,
        .simScgcEnBit = SIM_SCGC3_FTM2_ENABLE,
        .port         = FTM2_PORT,
        .pinQDPhA     = FTM2_QD_PHA_PIN,
        .pinQDPhB     = FTM2_QD_PHB_PIN,
        .pinQDMux     = FTM2_MUX_QD,
        .isr          = isrFtm2,
    },
};


volatile ftm_t *getFtmHandle(int timer);

static void callBackByMode(int timer)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    switch (ftmCtrl[timer].ftmMode) {
    case FTM_MODE_INPUT_CAPTURE:
        {
            int callBackValue = ftm->status & FTM_STATUS_MASK;
            if (callBackValue) {
                ftm->status &= ~callBackValue;
            }
            if (ftm->sc & FTM_SC_TOF_BIT) {
                callBackValue |= 1 << 8;
                ftm->sc &= ~FTM_SC_TOF_BIT;
            }
            ftmCtrl[timer].callBack(callBackValue);
        }
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        ftm->sc &= ~FTM_SC_TOF_BIT;
        if (ftm->qdctrl & FTM_QDCTRL_TOFDIR_BIT) {
            ftmCtrl[timer].callBack(1);
        } else {
            ftmCtrl[timer].callBack(-1);
        }
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        /* TODO Handle Me */
        break;
    case FTM_MODE_PWM:
        {
            int callBackValue = ftm->status & FTM_STATUS_MASK;
            if (callBackValue) {
                ftm->status &= ~callBackValue;
            }
            if (ftm->sc & FTM_SC_TOF_BIT) {
                callBackValue |= 1 << 8;
                ftm->sc &= ~FTM_SC_TOF_BIT;
                ftmCtrl[timer].callBack(callBackValue);
            }
        }
        break;
    default:
        assert(0);
        break;
    }
    return;
}
static void isrFtm0(void)
{
    callBackByMode(FTM_0);
    return;
}
static void isrFtm1(void)
{
    callBackByMode(FTM_1);
    return;
}

static void isrFtm2(void)
{
    callBackByMode(FTM_2);
    return;
}

volatile ftm_t *getFtmHandle(int timer)
{
    assert(timer >= 0);
    assert(timer < MAX_FTM);
    return ftmCtrl[timer].ftm;
}

void ftmSetQDPolarity(int timer, int invPolarity)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    uint32_t qdctrl = ftm->qdctrl;

    if (invPolarity) {
        qdctrl |=  FTM_QDCTRL_PHAPOL_BIT;
        qdctrl |= FTM_QDCTRL_PHBPOL_BIT;
    } else {
        qdctrl &= ~FTM_QDCTRL_PHAPOL_BIT;
        qdctrl &= ~FTM_QDCTRL_PHBPOL_BIT;

    }
    ftm->qdctrl = qdctrl;
}

void ftmSetQDFilter(int timer, uint8_t level)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    uint32_t qdctrl = ftm->qdctrl;

    if (level) {
        qdctrl |=  FTM_QDCTRL_PHAFLTREN_BIT;
        qdctrl |=  FTM_QDCTRL_PHBFLTREN_BIT;
    } else {
        qdctrl &= ~FTM_QDCTRL_PHAFLTREN_BIT;
        qdctrl &= ~FTM_QDCTRL_PHBFLTREN_BIT;
    }
    ftm->qdctrl = qdctrl;
    ftm->filter |= (level & FTM_FILTER_VALUE_MASK)
                                                  << FTM_FILTER_VALUE_CH1_SHIFT;
    ftm->filter |= (level & FTM_FILTER_VALUE_MASK)
                                                  << FTM_FILTER_VALUE_CH0_SHIFT;
}


uint16_t ftmRead(int timer)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    return ftm->cnt;
}

uint16_t ftmWrite(int timer, uint16_t mod, uint16_t initCount)
{
    volatile ftm_t *ftm = getFtmHandle(timer);

    ftm->mod   = mod;
    ftm->cntin = initCount;

    /* Writing any value to cnt resets to cntin */
    ftm->cnt = 0;

    return ftm->cnt;
}



static int portEnable(uint32_t port)
{
    int retVal = !ERROR;

    switch (port) {
    case PORTA: SIM_SCGC5 |= SIM_SCGC5_PORTA_ENABLE; break;
    case PORTB: SIM_SCGC5 |= SIM_SCGC5_PORTB_ENABLE; break;
    case PORTC: SIM_SCGC5 |= SIM_SCGC5_PORTC_ENABLE; break;
    case PORTD: SIM_SCGC5 |= SIM_SCGC5_PORTD_ENABLE; break;
    case PORTE: SIM_SCGC5 |= SIM_SCGC5_PORTE_ENABLE; break;
    default:
        assert(0);
        retVal = ERROR;
        break;
    }

    return retVal;
}


static uint16_t calcFreqPS(int32_t pwmFreq)
{
    int32_t clockHz = clockGetFreq(CLOCK_BUS);
    int32_t mod = (clockHz / pwmFreq) / 65536;
    int freqPS;

    assert(mod < 128);

    if (mod > 64) {
        freqPS = FTM_SC_PS_128;
    } else if (mod > 32) {
        freqPS = FTM_SC_PS_64;
    } else if (mod > 16) {
        freqPS = FTM_SC_PS_32;
    } else if (mod > 8) {
        freqPS = FTM_SC_PS_16;
    } else if (mod > 4) {
        freqPS = FTM_SC_PS_8;
    } else if (mod > 2) {
        freqPS = FTM_SC_PS_4;
    } else if (mod > 1) {
        freqPS = FTM_SC_PS_2;
    } else {
        freqPS = FTM_SC_PS_1;
    }
    return freqPS;
};
static uint16_t freqToMod(int32_t pwmFreq, int32_t freqPS)
{
    int32_t clockHz = clockGetFreq(CLOCK_BUS);
    uint32_t mod;

    switch (freqPS) {
    default:
    case FTM_SC_PS_1:
        mod = (uint16_t) (clockHz / pwmFreq);
        break;
    case FTM_SC_PS_2:
        mod = (uint16_t) (clockHz / (2 *pwmFreq));
        break;
    case FTM_SC_PS_4:
        mod = (uint16_t) (clockHz / (4 *pwmFreq));
        break;
    case FTM_SC_PS_8:
        mod = (uint16_t) (clockHz / (8 *pwmFreq));
        break;
    case FTM_SC_PS_16:
        mod = (uint16_t) (clockHz / (16 *pwmFreq));
        break;
    case FTM_SC_PS_32:
        mod = (uint16_t) (clockHz / (32 *pwmFreq));
        break;
    case FTM_SC_PS_64:
        mod = (uint16_t) (clockHz / (64 *pwmFreq));
        break;
    case FTM_SC_PS_128:
        mod = (uint16_t) (clockHz / (128 *pwmFreq));
        break;
    }
    assert(mod < 0xFFFF);
    return (uint16_t)mod;
}

static uint16_t dutyToCv(int32_t dutyScaled, uint16_t mod)
{

    if (dutyScaled < 0) {
        dutyScaled = 0;
    }
    if (dutyScaled > 32768) {
        dutyScaled = 32768;
    }

    return (uint16_t) ((mod * dutyScaled)/32768);
}

void ftmPwmWrite(int timer, int ch, int32_t dutyScaled)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    volatile uint32_t *cvPtr;

    switch (ch) {
    case FTM_CH_0:
        cvPtr  = &ftm->c0v;
        break;
    case FTM_CH_1:
        cvPtr  = &ftm->c1v;
        break;
    case FTM_CH_2:
        break;
    case FTM_CH_3:
        break;
    case FTM_CH_4:
        break;
    case FTM_CH_5:
        break;
    case FTM_CH_6:
        break;
    case FTM_CH_7:
        break;
    default:
        assert(0);
        cvPtr  = NULL;
        break;
    }
    if (cvPtr) {
        ftm->cnt = 0;
        ftm->mode  = FTM_MODE_WPDIS_BIT;
        *cvPtr  = dutyToCv(dutyScaled, ftm->mod);
        ftm->pwmload |= FTM_PWMLOAD_LDOK_BIT;
    }

}

void ftmInit(int timer, void *callBack, ftmCfg_t *ftmCfg)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    uint8_t chIdx;
    int freqPS;
    *ftmCtrl[timer].simScgcPtr |= ftmCtrl[timer].simScgcEnBit;
    if (callBack) {
        ftmCtrl[timer].callBack = callBack;
        hwInstallISRHandler(ISR_FTM0 + timer, ftmCtrl[timer].isr);
        ftm->sc |= FTM_SC_TOIE_BIT;
    }

    ftm->conf |= FTM_CONF_BDMMODE_FUNCTIONAL << FTM_CONF_BDMMODE_SHIFT;
    ftm->mod   = ftmCfg->mod;
    ftm->cntin = ftmCfg->initCount;
    ftmCtrl[timer].ftmMode = ftmCfg->mode;
    switch (ftmCfg->mode) {
    case FTM_MODE_INPUT_CAPTURE:
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        if (timer != FTM_0) {
            /* TODO Add QD support on FTM_0 - check pinout */
            ftm->sc |= FTM_SC_CLKS_EXTERNAL_CLOCK << FTM_SC_CLKS_SHIFT;
            if (portEnable(ftmCtrl[timer].port) != ERROR) {
                ftm->qdctrl = FTM_QDCTRL_QUADEN_BIT;
                PORT_PCR(ftmCtrl[timer].port, ftmCtrl[timer].pinQDPhA)
                    = PORT_IRQC_DISABLED | ftmCtrl[timer].pinQDMux;
                PORT_PCR(ftmCtrl[timer].port, ftmCtrl[timer].pinQDPhB)
                                 = PORT_IRQC_DISABLED | ftmCtrl[timer].pinQDMux;
            }
        }
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        break;
    case FTM_MODE_PWM:
        freqPS = calcFreqPS(ftmCfg->pwmFreq);
        ftm->sc |= (FTM_SC_CLKS_BUS << FTM_SC_CLKS_SHIFT) | freqPS;
        for (chIdx = FTM_CH_0; chIdx < MAX_FTM_CH; chIdx++) {
            volatile uint32_t *ccsPtr;
            volatile uint32_t *cvPtr;
            uint32_t mod;
            if (ftmCfg->channels[chIdx] == FTM_CH_NONE) {
                break;
            }

            if (ftmCfg->channels[chIdx] < FTM_CH_4) {
                /* TODO Add upper channel support - if pins support */
                if (portEnable(ftmCtrl[timer].port) != ERROR) {
                    PORT_PCR(ftmCtrl[timer].port,
                            ftmCtrl[timer].pinCh[ftmCfg->channels[chIdx]])
                        = PORT_IRQC_DISABLED | ftmCtrl[timer].pinChMux;
                }
            }
            ftm->cnt = 0;
            ftm->mode  = FTM_MODE_WPDIS_BIT;
            /* need local mod as you can't read ftm->mod until it is latched over */
            ftm->mod   = mod = freqToMod(ftmCfg->pwmFreq, freqPS);
            ftm->cntin = 0;
            ftm->qdctrl &= ~FTM_QDCTRL_QUADEN_BIT;
            switch (ftmCfg->channels[chIdx]) {
            case FTM_CH_0:
                ccsPtr = &ftm->c0cs;
                cvPtr  = &ftm->c0v;
               break;
            case FTM_CH_1:
                ccsPtr = &ftm->c1cs;
                cvPtr  = &ftm->c1v;
                break;
            case FTM_CH_2:
                break;
            case FTM_CH_3:
                break;
            case FTM_CH_4:
                break;
            case FTM_CH_5:
                break;
            case FTM_CH_6:
                break;
            case FTM_CH_7:
                break;
            default:
                assert(0);
                ccsPtr = NULL;
                cvPtr  = NULL;
                break;
            }
            if (ccsPtr && cvPtr) {
                *ccsPtr = FTM_CH_CS_MSB_BIT | FTM_CH_CS_ELSB_BIT;
#if 0
                /*
                 * TODO: It is possible to interrupt on each channel
                 * but this quickly swamps the processor with interrupts.
                 * I would like to add an implicit request for the
                 * ch interrupts as separate command.
                 *
                 * Channel interrupts probably make more sense for the
                 * input capture mode.
                 *
                 */
                if (callBack) { // Maybe limit to first ch?  && chIdx == 0) {
                    *ccsPtr |= FTM_CH_CS_CHIE_BIT;
                }
#endif
                *cvPtr  = dutyToCv(ftmCfg->dutyScaled[chIdx], mod);
                ftm->pwmload |= FTM_PWMLOAD_LDOK_BIT;
            }

        }
        break;
    }
    ftm->mode |= FTM_MODE_FTMEN_BIT;
}


