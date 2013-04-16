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


#define FTM0_PWM0_PORT PORTC
#define FTM0_PWM0_PIN 1
#define FTM0_PWM0_MUX (PORT_MUX_ALT4)

#define FTM0_PWM1_PORT PORTC
#define FTM0_PWM1_PIN 2
#define FTM0_PWM1_MUX (PORT_MUX_ALT4)

#define FTM0_PWM2_PORT PORTC
#define FTM0_PWM2_PIN 3
#define FTM0_PWM2_MUX (PORT_MUX_ALT4)

#define FTM0_PWM3_PORT PORTA
#define FTM0_PWM3_PIN 6
#define FTM0_PWM3_MUX (PORT_MUX_ALT3)

#define FTM0_PWM4_PORT PORTA
#define FTM0_PWM4_PIN 7
#define FTM0_PWM4_MUX (PORT_MUX_ALT3)

#define FTM0_PWM5_PORT PORTD
#define FTM0_PWM5_PIN 5
#define FTM0_PWM5_MUX (PORT_MUX_ALT4)





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
    unsigned pinCh[8];
    unsigned pinChMux[8];
    unsigned port[8];
    void (*isr)(void);
    int  ftmMode;
    uint32_t cfgBits;
    int32_t pwmAlign;
    void (*callBack)(int32_t index);
} ftmCtrl_t;

enum {
    PWM_EDGE_ALIGNED,
    PWM_CENTER_ALIGNED,
};
enum {
    PWM_COMBINED_CHS_0_1_IDX,
    PWM_COMBINED_CHS_2_3_IDX,
    PWM_COMBINED_CHS_4_5_IDX,
    PWM_COMBINED_CHS_6_7_IDX,
};

static void isrFtm0(void);
static void isrFtm1(void);
static void isrFtm2(void);

static ftmCtrl_t ftmCtrl[MAX_FTM] = {
    [FTM_0] = {
        .ftm = ((volatile ftm_t * const) FTM0_CTRL_BASE),
        .simScgcPtr   = SIM_SCGC6_PTR,
        .simScgcEnBit = SIM_SCGC6_FTM0_ENABLE,
        .port         = { FTM0_PWM0_PORT, FTM0_PWM1_PORT, FTM0_PWM2_PORT,
                          FTM0_PWM3_PORT, FTM0_PWM4_PORT, FTM0_PWM5_PORT },
        .pinCh        = { FTM0_PWM0_PIN,  FTM0_PWM1_PIN,  FTM0_PWM2_PIN,
                          FTM0_PWM3_PIN,  FTM0_PWM4_PIN,  FTM0_PWM5_PIN},
        .pinChMux     = { FTM0_PWM0_MUX,  FTM0_PWM1_MUX,  FTM0_PWM2_MUX,
                          FTM0_PWM3_MUX,  FTM0_PWM4_MUX,  FTM0_PWM5_MUX},
        .isr          = isrFtm0,
    },
    [FTM_1] = {
        .ftm = ((volatile ftm_t * const) FTM1_CTRL_BASE),
        .simScgcPtr   = SIM_SCGC6_PTR,
        .simScgcEnBit = SIM_SCGC6_FTM1_ENABLE,
        .port         = { FTM1_PORT },
        .pinQDPhA     = FTM1_QD_PHA_PIN,
        .pinQDPhB     = FTM1_QD_PHB_PIN,
        .pinQDMux     = FTM1_MUX_QD,
        .isr          = isrFtm1,
    },
    [FTM_2] = {
        .ftm = ((volatile ftm_t * const) FTM2_CTRL_BASE),
        .simScgcPtr   = SIM_SCGC3_PTR,
        .simScgcEnBit = SIM_SCGC3_FTM2_ENABLE,
        .port         = { FTM2_PORT },
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

    ftm->sync |= FTM_SYNC_SWSYNC_BIT;

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

static uint32_t deadCounts(uint32_t deadTime)
{
    uint8_t  dtps;
    uint32_t clockCounts  = deadTime * (clockGetFreq(CLOCK_BUS) / 10000000);

    if (clockCounts > 63) {
        dtps = FTM_DEADTIME_DTPS_4;
        clockCounts /= 4;
    }

    if (clockCounts > 63) {
        dtps = FTM_DEADTIME_DTPS_16;
        clockCounts /= 4;
    }

    if (clockCounts > 63) {
        clockCounts = 63;
    }

    if (clockCounts < 2) {
        clockCounts = 2;
    }

    clockCounts |= dtps << FTM_DEADTIME_DTPS_SHIFT;

    return clockCounts;
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
}
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

void ftmSetOutputMask(int timer, uint32_t mask, int32_t sync)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    ftm->outmask = mask;
    if (sync) {
        ftm->sync |= FTM_SYNC_SWSYNC_BIT;
    }
}


void ftmSetInvCtrl(int timer, uint32_t mask, int32_t sync)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    ftm->invctrl = mask;
    if (sync) {
        ftm->sync |= FTM_SYNC_SWSYNC_BIT;
    }
}


void ftmSetOutput(int timer, int ch, int setOn)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    uint32_t value = ftm->swoctrl;

    if (ftm->combine & FTM_COMBINED_BIT && ch < FTM_CH_NONE) {

        assert(ch < MAX_FTM_CH);

        value |= (FTM_SWOCTRL_OC_BIT << ch);
        if (setOn) {
            value |= (FTM_SWOCTRL_OCV_BIT << ch);
        } else {
            value &= ~(FTM_SWOCTRL_OCV_BIT << ch);
        }
    } else {
        value &= ~(FTM_SWOCTRL_OC_BIT << ch);
    }
    ftm->swoctrl = value;
}

void ftmPwmWrite(int timer, int ch, int32_t dutyScaled, int32_t sync)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    volatile uint32_t *cvPtr;
    volatile uint32_t *cvPtrCombo = NULL;
    int32_t master;

    switch (ch) {
    case FTM_CH_0:
        cvPtr = &ftm->c0v;
        if (ftm->combine & FTM_COMBINED_BIT) {
            cvPtrCombo  = &ftm->c1v;
            master = FALSE;
        }
        break;
    case FTM_CH_1:
        cvPtr = &ftm->c1v;
        if (ftm->combine & FTM_COMBINED_BIT) {
            cvPtrCombo  = &ftm->c0v;
            master = TRUE;
        }
        break;
    case FTM_CH_2:
        cvPtr = &ftm->c2v;
        if (ftm->combine & (FTM_COMBINED_BIT << 8)) {
            cvPtrCombo  = &ftm->c3v;
            master = FALSE;
        }
        break;
    case FTM_CH_3:
        cvPtr = &ftm->c3v;
        if (ftm->combine & (FTM_COMBINED_BIT << 8)) {
            cvPtrCombo  = &ftm->c2v;
            master = TRUE;
        }
        break;
    case FTM_CH_4:
        cvPtr = &ftm->c4v;
        if (ftm->combine & (FTM_COMBINED_BIT << 16)) {
            cvPtrCombo  = &ftm->c5v;
            master = FALSE;
        }
        break;
    case FTM_CH_5:
        cvPtr = &ftm->c5v;
        if (ftm->combine & (FTM_COMBINED_BIT << 16)) {
            cvPtrCombo  = &ftm->c4v;
            master = TRUE;
        }
        break;
    case FTM_CH_6:
        cvPtr = &ftm->c6v;
        if (ftm->combine & (FTM_COMBINED_BIT << 24)) {
            cvPtrCombo  = &ftm->c7v;
            master = FALSE;
        }
        break;
    case FTM_CH_7:
        cvPtr = &ftm->c7v;
        if (ftm->combine & (FTM_COMBINED_BIT << 24)) {
            cvPtrCombo  = &ftm->c6v;
            master = TRUE;
        }
        break;
    default:
        assert(0);
        cvPtr  = NULL;
        break;
    }
    if (cvPtr) {
        uint16_t cv = dutyToCv(dutyScaled, ftm->mod);
        uint16_t altCv;
        if (ftmCtrl[timer].pwmAlign == PWM_CENTER_ALIGNED) {
            altCv = -cv;
        } else {
            altCv = 0;
        }
        if (cvPtrCombo) {
            if (master) {
                *cvPtrCombo = altCv;
                *cvPtr  = cv;
            } else {
                *cvPtr = altCv;
                *cvPtrCombo = cv;
            }
        } else {
            *cvPtr  = cv;
        }
        if (sync) {
            ftm->sync |= FTM_SYNC_SWSYNC_BIT;
        }
    }

}

void ftmInit(int timer, void *callBack, ftmCfg_t *ftmCfg)
{
    volatile ftm_t *ftm = getFtmHandle(timer);
    uint8_t chIdx;
    int freqPS;
    uint16_t mod;
    uint16_t cntin;
    uint32_t outmask = 0;
    uint32_t value;

    *ftmCtrl[timer].simScgcPtr |= ftmCtrl[timer].simScgcEnBit;
#if 1
    if (callBack) {
        ftmCtrl[timer].callBack = callBack;
        hwInstallISRHandler(ISR_FTM0 + timer, ftmCtrl[timer].isr);
        ftm->sc |= FTM_SC_TOIE_BIT;
    }
#endif
    ftm->conf |= FTM_CONF_BDMMODE_FUNCTIONAL << FTM_CONF_BDMMODE_SHIFT;
    ftm->cntin = ftmCfg->initCount;
    ftmCtrl[timer].ftmMode = ftmCfg->mode;

    ftm->outmask = FTM_OUTMASK_ALL;
    switch (ftmCfg->mode) {
    case FTM_MODE_INPUT_CAPTURE:
        break;
    case FTM_MODE_QUADRATURE_DECODE:
        ftm->mod   = ftmCfg->mod;
        if (timer != FTM_0) {
            /* TODO Add QD support on FTM_0 - check pinout */
            ftm->sc |= FTM_SC_CLKS_EXTERNAL_CLOCK << FTM_SC_CLKS_SHIFT;
            if (portEnable(ftmCtrl[timer].port[0]) != ERROR) {
                ftm->qdctrl = FTM_QDCTRL_QUADEN_BIT;
                PORT_PCR(ftmCtrl[timer].port[0], ftmCtrl[timer].pinQDPhA)
                                 = PORT_IRQC_DISABLED | ftmCtrl[timer].pinQDMux;
                PORT_PCR(ftmCtrl[timer].port[0], ftmCtrl[timer].pinQDPhB)
                                 = PORT_IRQC_DISABLED | ftmCtrl[timer].pinQDMux;
            }
            switch (ftmCfg->quadCfg) {
            default:
            case FTM_QUAD_CONFIG_FILTER_NONE:
                ftm->qdctrl &= ~(FTM_QDCTRL_PHAFLTREN_BIT
                               | FTM_QDCTRL_PHBFLTREN_BIT);
                break;
            case FTM_QUAD_CONFIG_FILTER_LOW:
            case FTM_QUAD_CONFIG_FILTER_MED:
            case FTM_QUAD_CONFIG_FILTER_HIGH:
                ftm->qdctrl |= FTM_QDCTRL_PHAFLTREN_BIT;
                ftm->qdctrl |= FTM_QDCTRL_PHBFLTREN_BIT;
                value  = ftmCfg->quadCfg << FTM_FILTER_VALUE_CH0_SHIFT;
                value |= ftmCfg->quadCfg << FTM_FILTER_VALUE_CH1_SHIFT;
                ftm->filter = value;
                break;
            }
        }
        break;
    case FTM_MODE_OUTPUT_COMPARE:
        break;
    case FTM_MODE_PWM:
        freqPS = calcFreqPS(ftmCfg->pwmFreq);

        if (ftmCfg->pwmCfgBits & FTM_PWM_CFG_CENTER_ALINGNED) {
            ftmCtrl[timer].pwmAlign = PWM_CENTER_ALIGNED;
            /* The Freescale ref manual indicates that one should set
             * the FTM_SC_CPWMS_BIT for center aligned PWM.  However, this
             * doesn't work in combined mode (the channels setup the realtive
             * triggering to center the pulse).
             */
            if (ftmCfg->pwmCfgBits & FTM_PWM_CFG_COMBINED_MASK) {
                ftm->sc &= ~FTM_SC_CPWMS_BIT;
            } else {
                ftm->sc |=  FTM_SC_CPWMS_BIT;
            }
        } else {
            ftmCtrl[timer].pwmAlign = PWM_EDGE_ALIGNED;
            ftm->sc &= ~FTM_SC_CPWMS_BIT;
        }
        ftm->synconf =  ( FTM_SYNCONF_SYNCMODE_BIT
                        | FTM_SYNCONF_SWWRBUF_BIT
                        | FTM_SYNCONF_INVC_BIT | FTM_SYNCONF_SWINVC_BIT);
        ftm->sync  =  FTM_SYNC_CNTMAX_BIT;
        ftm->sync |=  FTM_SYNC_SWSYNC_BIT;
        if (ftmCfg->pwmCfgBits & FTM_PWM_CFG_OUTPUT_MASK) {
            ftm->synconf |= FTM_SYNCONF_SWOM_BIT;
            ftm->sync |= FTM_SYNC_SYNCHOM_BIT;
        }
        if (ftmCfg->deadTime) {
            ftm->mode  = FTM_MODE_WPDIS_BIT;
            ftm->deadtime = deadCounts(ftmCfg->deadTime);
        }

        /* need local mod as you can't read ftm->mod until it is latched over */
        mod = freqToMod(ftmCfg->pwmFreq, freqPS);
        if (ftmCtrl[timer].pwmAlign == PWM_CENTER_ALIGNED) {
            cntin = -mod / 2;
            mod   =  mod / 2 - 1;
        } else {
            mod   = mod - 1;
            cntin = 0;
        }

        ftm->mod   = mod;
        ftm->cntin = cntin;

        ftm->qdctrl &= ~FTM_QDCTRL_QUADEN_BIT;


        ftm->exttrig = ftmCfg->triggerBits;


        for (chIdx = FTM_CH_0; chIdx < MAX_FTM_CH; chIdx++) {
            volatile uint32_t *ccsPtr;
            volatile uint32_t *cvPtr;
            int32_t combinedBit;
            int32_t complementaryBit;
            int32_t combinedIdx;
            uint8_t channel   = ftmCfg->channels[chIdx] ;
            int     activeLow = ftmCfg->activeLow[chIdx];

            if (channel == FTM_CH_NONE) {
                break;
            }
            if (portEnable(ftmCtrl[timer].port[channel]) == ERROR) {
                assert(0);
                break;
            }
            PORT_PCR(ftmCtrl[timer].port[channel],
                                                  ftmCtrl[timer].pinCh[channel])
                                 = PORT_IRQC_DISABLED
                                   | ftmCtrl[timer].pinChMux[channel];

            ftm->cnt = 0;
            ftm->mode  = FTM_MODE_WPDIS_BIT;
            if (activeLow) {
                ftm->pol |=  (1 << channel);
            } else {
                ftm->pol &= ~(1 << channel);
            }

            if (ftmCfg->pwmCfgBits & FTM_PWM_CFG_OUTPUT_MASK) {
                outmask |=  (1 << channel);
            } else {
                outmask &= ~(1 << channel);
            }

            switch (channel) {
            case FTM_CH_0:
                ccsPtr = &ftm->c0cs;
                cvPtr  = &ftm->c0v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_0_1;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_0_1;
                combinedIdx      = PWM_COMBINED_CHS_0_1_IDX;
               break;
            case FTM_CH_1:
                ccsPtr = &ftm->c1cs;
                cvPtr  = &ftm->c1v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_0_1;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_0_1;
                combinedIdx      = PWM_COMBINED_CHS_0_1_IDX;
                break;
            case FTM_CH_2:
                ccsPtr = &ftm->c2cs;
                cvPtr  = &ftm->c2v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_2_3;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_2_3;
                combinedIdx      = PWM_COMBINED_CHS_2_3_IDX;
                break;
            case FTM_CH_3:
                ccsPtr = &ftm->c3cs;
                cvPtr  = &ftm->c3v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_2_3;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_2_3;
                combinedIdx      = PWM_COMBINED_CHS_2_3_IDX;
                break;
            case FTM_CH_4:
                ccsPtr = &ftm->c4cs;
                cvPtr  = &ftm->c4v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_4_5;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_4_5;
                combinedIdx      = PWM_COMBINED_CHS_4_5_IDX;
                break;
            case FTM_CH_5:
                ccsPtr = &ftm->c5cs;
                cvPtr  = &ftm->c5v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_4_5;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_4_5;
                combinedIdx      = PWM_COMBINED_CHS_4_5_IDX;
                break;
            case FTM_CH_6:
                ccsPtr = &ftm->c6cs;
                cvPtr  = &ftm->c6v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_6_7;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_6_7;
                combinedIdx      = PWM_COMBINED_CHS_6_7_IDX;
                break;
            case FTM_CH_7:
                ccsPtr = &ftm->c7cs;
                cvPtr  = &ftm->c7v;
                combinedBit      = FTM_PWM_CFG_COMBINED_MODE_CHS_6_7;
                complementaryBit = FTM_PWM_CFG_COMPLEMENTARY_CH_6_7;
                combinedIdx      = PWM_COMBINED_CHS_6_7_IDX;
                break;
            default:
                assert(0);
                ccsPtr = NULL;
                cvPtr  = NULL;
                break;
            }

            if (ccsPtr && cvPtr) {
                if (ftmCtrl[timer].pwmAlign == PWM_CENTER_ALIGNED) {
                    *ccsPtr = FTM_CH_CS_ELSB_BIT;
                } else {
                    *ccsPtr = FTM_CH_CS_MSB_BIT | FTM_CH_CS_ELSB_BIT;
                }
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
                if (callBack) { // Maybe limit to first ch?  && chIdx == 0)
                    *ccsPtr |= FTM_CH_CS_CHIE_BIT;
                }
#endif


                if (ftmCfg->pwmCfgBits & combinedBit) {
                    uint32_t combine = ftm->combine;

                    combine |= (FTM_COMBINED_BIT << (8 * combinedIdx));
                    combine |= (FTM_SYNCEN_BIT   << (8 * combinedIdx));
                    if (ftmCfg->pwmCfgBits & complementaryBit) {
                        combine |=  (FTM_COMP_BIT << (8 * combinedIdx));
                    } else {
                        combine &= ~(FTM_COMP_BIT << (8 * combinedIdx));
                    }
                    if (ftmCfg->deadTime) {
                        ftm->mode  = FTM_MODE_WPDIS_BIT;
                        combine |= (FTM_DEADTIME_BIT << (8 * combinedIdx));
                    }
                    ftm->mode  = FTM_MODE_WPDIS_BIT;
                    ftm->combine = combine;

                } else {
                    ftm->mode  = FTM_MODE_WPDIS_BIT;
                    ftm->combine &= ~(FTM_COMBINED_BIT << (8 * combinedIdx));
                    ftm->combine &= ~(FTM_COMP_BIT << (8 * combinedIdx));
                    ftm->combine |= (FTM_SYNCEN_BIT   << (8 * combinedIdx));
                }

                ftm->mode  = FTM_MODE_WPDIS_BIT;
                ftmPwmWrite(timer, channel, ftmCfg->dutyScaled[chIdx], TRUE);
            }
        }
        ftm->mode |= FTM_MODE_INIT_BIT;
        ftm->outmask = outmask;
        ftm->pwmload |= FTM_PWMLOAD_LDOK_BIT | FTM_PWMLOAD_ALL_MASK;
        ftm->sync |= FTM_SYNC_SWSYNC_BIT;
        ftm->sc |= (FTM_SC_CLKS_BUS << FTM_SC_CLKS_SHIFT) | freqPS;
        break;
    }
    ftm->mode |= FTM_MODE_FTMEN_BIT;
}


