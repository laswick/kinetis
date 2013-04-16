/*******************************************************************************
*
* demoBldcSin.c
*
* James McAnanama
*
* This demo drives the a BLDC motor using the TWR-MC-LV3PH board.  Here we are
* doing space vector modulated sinusoidal drive using a pid loop for position
* control.
*
* Attach a qaud encoder to FTM_2.
*
* Attach a pot to ADC0_DP0.
*
* Configure the motor/encoder defines (ELEC_CYCLES_PER_MECH_CYCLE to end of sinLUT).
*
* Give'er.
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



                                        /* ADC - Use pot for postion setpoint */
static scaled_t angleSetPoint;

/******************************************************************************
* potCallBackHandler
*
* Parse RX messages.
*
******************************************************************************/
static void potCallBackHandler(int32_t adcReg, uint32_t const *buf, int len)
{
    if (len) {
        len--;
                                  /* (value/PGA gain)/resolution * full scale */
        angleSetPoint = (buf[len] / (1 * 4095.0) * 360) * UNITY;
    }
   return;
}


                                                   /* Space Vector Modulation */
enum {
    SVM_VECTOR_0   =        0,
    SVM_VECTOR_60  =        60  * UNITY,
    SVM_VECTOR_120 =        120 * UNITY,
    SVM_VECTOR_180 =        180 * UNITY,
    SVM_VECTOR_240 =        240 * UNITY,
    SVM_VECTOR_300 =        300 * UNITY,
};
#define SVM_60_DEGREES  (60 * UNITY)

/* The Hurst motor has an optical encoder 1000 counts per mechanical revolution
 * and 5 electrical cycles per mechanical revolution.  So, there are
 * 1000 / 5 * 60/360 = 33 counts for every 60 degrees of the electrical cycle.
 * We need a sin LUT with 33 entries for 60 degrees.
 */
#define ELEC_CYCLES_PER_MECH_CYCLE 5
#define COUNTS_PER_360_MECH_DEG   1000
#define COUNTS_PER_360_ELEC_DEG  (COUNTS_PER_360_MECH_DEG \
                                / ELEC_CYCLES_PER_MECH_CYCLE)
#define COUNTS_PER_60_ELEC_DEG   33
#define MAX_DRIVE   (0.3 * UNITY)
#define MAX_PHASE_ADVANCE (30 * UNITY)

static const int32_t sinLUT[] = {
        0,      1040,   2078,   3115,   4148,
        5177,   6201,   7219,   8230,   9232,
        10225,  11207,  12179,  13138,  14084,
        15015,  15932,  16832,  17716,  18581,
        19428,  20256,  21063,  21849,  22613,
        23354,  24071,  24764,  25433,  26076,
        26692,  27282,  27844,  28378
};


typedef struct {
    int      sinLUTIdx;
    scaled_t pwmADuty;
    scaled_t pwmBDuty;
    scaled_t pwmCDuty;
} svm_t;
static svm_t svmOutput;


/*******************************************************************************
 *
 *
 *   U120 ___ U60
 *      /\   /\
 * U180/__\ /__\ U0
 *     \  / \  /
 *      \/___\/
 *  U240     U300
 *
 *  Vector  |  PWMA  | PWMB | PWMC
 *   U0     |   1    |  0   |  0
 *   U60    |   1    |  1   |  0
 *   U120   |   0    |  1   |  0
 *   U180   |   0    |  1   |  1
 *   U240   |   0    |  0   |  1
 *   U300   |   1    |  0   |  1
 *
 *  output  - Full output duty cycle in %FS.
 *  angle   - Electrical angle scaled
 *
 *  returns svm relative duty cyles for each phase, svm_t.
 *
 ******************************************************************************/
static int svmDuty(scaled_t duty[3], scaled_t relativeAngle[2], scaled_t output)
{
    int i;
    scaled_t dutySum = 0;
    int idx;

    for (i = 0; i < 2; i++) {
                                                                /* sin(theta) */
        idx = divS(relativeAngle[i] * COUNTS_PER_60_ELEC_DEG, SVM_60_DEGREES);
        idx = SCALE_TO_INT(idx, 15);
        duty[i] = sinLUT[idx];
    }

    for (i = 0; i < 2; i++) {
                                 /* Vector i duty cycle = output * sin(theta) */
        duty[i] = duty[i] * output / UNITY;
        dutySum += duty[i];
    }
                                  /* Store half the remainder of the %pwm period
                                   * for use across the channels */
    duty[2] = (UNITY - dutySum) / 2;

    return idx;
}
static svm_t resolveSVM(scaled_t output, scaled_t angle)
{
    svm_t svm;
    scaled_t relativeAngle[2];
    scaled_t duty[3]; /* [duty_1, duty_2, half duty_0] */


    angle %= 360 * UNITY;

    if (angle <= SVM_VECTOR_60) {
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
        relativeAngle[1] = angle;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output);
        svm.pwmADuty = duty[0] + duty[1] + duty[2];
        svm.pwmBDuty =           duty[1] + duty[2];
        svm.pwmCDuty =                     duty[2];
    } else if (angle <= SVM_VECTOR_120) {
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

        relativeAngle[1] = angle - SVM_VECTOR_60;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output);
        svm.pwmADuty = duty[0]           + duty[2];
        svm.pwmBDuty = duty[0] + duty[1] + duty[2];
        svm.pwmCDuty =                     duty[2];
    } else if (angle <= SVM_VECTOR_180) {
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

        relativeAngle[1] = angle - SVM_VECTOR_120;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output);
        svm.pwmADuty =                     duty[2];
        svm.pwmBDuty = duty[0] + duty[1] + duty[2];
        svm.pwmCDuty =           duty[1] + duty[2];
    } else if (angle < SVM_VECTOR_240) {
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

        relativeAngle[1] = angle - SVM_VECTOR_180;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output);
        svm.pwmADuty =                     duty[2];
        svm.pwmBDuty = duty[0]           + duty[2];
        svm.pwmCDuty = duty[0] + duty[1] + duty[2];
    } else if (angle < SVM_VECTOR_300) {
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

        relativeAngle[1] = angle - SVM_VECTOR_240;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output);
        svm.pwmADuty =           duty[1] + duty[2];
        svm.pwmBDuty =                     duty[2];
        svm.pwmCDuty = duty[0] + duty[1] + duty[2];
    } else {
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

        relativeAngle[1] = angle - SVM_VECTOR_300;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output);
        svm.pwmADuty = duty[0] + duty[1] + duty[2];
        svm.pwmBDuty =                     duty[2];
        svm.pwmCDuty = duty[0]           + duty[2];
    }

    return svm;
}

static int updateFlags;
enum {
    UPDATE_NONE         = 0,
    UPDATE_HALT         = 1 << 1,
};

                                                                /* FET DRIVER */
#define FET_DRIVER_RESET_PORT PORTE
#define FET_DRIVER_RESET_PIN  27
#define FET_DRIVER_ENABLE_PORT PORTE
#define FET_DRIVER_ENABLE_PIN  28
#define IRQ_PORT PORTA
#define IRQ_PIN 27

#if 0

#if 1
#define HALL_C_PORT PORTA
#define HALL_C_PIN 8
#define HALL_B_PORT PORTA
#define HALL_B_PIN 9
#define HALL_A_PORT PORTD
#define HALL_A_PIN 6
#else
#define HALL_A_PORT PORTA
#define HALL_A_PIN 8
#define HALL_B_PORT PORTA
#define HALL_B_PIN 9
#define HALL_C_PORT PORTD
#define HALL_C_PIN 6
#endif

#endif
#define ENCODER_INDEX_PORT PORTB
#define ENCODER_INDEX_PIN  20


enum {
   NULL_CMD     = 0x00,
   MASK0_CMD    = 0x20,
   MASK1_CMD    = 0x30,
   MODE_CMD     = 0x40,
   CLINT0_CMD   = 0x60,
   CLINT1_CMD   = 0x70,
   DEADTIME_CMD = 0x80,
};

enum {
    NULL_CMD_GET_REG0,
    NULL_CMD_GET_REG1,
    NULL_CMD_GET_REG2,
    NULL_CMD_GET_REG3,
    MAX_NULL_CMD_REG,
};

enum {
    MASK0_OVERTEMP_BIT          = 1 << 0,
    MASK0_DESATURATION_BIT      = 1 << 1,
    MASK0_VLS_UNDER_VOLTAGE_BIT = 1 << 2,
    MASK0_OVER_CURRENT_BIT      = 1 << 3,
};

enum {
    MASK1_PHASE_ERROR_BIT   = 1 << 0,
    MASK1_FRAMING_ERROR_BIT = 1 << 1,
    MASK1_WRITE_ERROR_BIT   = 1 << 2,
    MASK1_RESET_EVENT_BIT   = 1 << 3,
};

enum {
    MODE_MODE_LOCK_BIT          = 1 << 0,
    MODE_FULLON_BIT             = 1 << 1,

    MODE_DESATURATION_FAULT_BIT = 1 << 3,
};

/*
 * To set deadtime, send the deadtime cmd, then hold the chip
 * select for 16 times longer than the desired deadtime.  POR
 * value is 255 internal time base cycles (~15uSec).
 *
 * Calling the deadtime cmd with no cal bit sets deadtime to zero.
 *
 */
enum {
    DEADTIME_CALIBRATE_BIT      = 1 << 0,
};


                                           /* STATUS_REG0 - Status Latch Bits */
enum {
    STATUS_REG0_OVER_TEMP     = 1 << 0,
    STATUS_REG0_DESATURATION  = 1 << 1,
    STATUS_REG0_LSV           = 1 << 2,
    STATUS_REG0_OVER_CURRENT  = 1 << 3,
    STATUS_REG0_PHASE_ERROR   = 1 << 4,
    STATUS_REG0_FRAMING_ERROR = 1 << 5,
    STATUS_REG0_WRITE_ERROR   = 1 << 6,
    STATUS_REG0_RESET_EVENT   = 1 << 7,
};

                                                   /* STATUS_REG1 - Mode bits */
enum {
    STATUS_REG1_MODE_LOCK            = 1 << 0,
    STATUS_REG1_MODE_FULLON          = 1 << 1,

    STATUS_REG1_DEADTIME_CAL         = 1 << 3,
    STATUS_REG1_CALIBRATION_OVERFLOW = 1 << 4,
    STATUS_REG1_ZERO_DEADTIME        = 1 << 5,
    STATUS_REG1_DESATURATION_MODE    = 1 << 6,
};

                                                   /* STATUS_REG2 - Mask bits */
enum {
    STATUS_REG2_MASK0_0     = 1 << 0,
    STATUS_REG2_MASK0_1     = 1 << 1,
    STATUS_REG2_MASK0_2     = 1 << 2,
    STATUS_REG2_MASK0_3     = 1 << 3,
    STATUS_REG2_MASK1_0     = 1 << 4,
    STATUS_REG2_MASK1_1     = 1 << 5,
    STATUS_REG2_MASK1_2     = 1 << 6,
    STATUS_REG2_MASK1_3     = 1 << 7,
};




#define MASK_PHASE_A_HIGH FTM_OUTMASK_CH0_OI_BIT
#define MASK_PHASE_A_LOW  FTM_OUTMASK_CH1_OI_BIT

#define MASK_PHASE_B_HIGH FTM_OUTMASK_CH2_OI_BIT
#define MASK_PHASE_B_LOW  FTM_OUTMASK_CH3_OI_BIT

#define MASK_PHASE_C_HIGH FTM_OUTMASK_CH4_OI_BIT
#define MASK_PHASE_C_LOW  FTM_OUTMASK_CH5_OI_BIT

#define MASK_HIGH_SIDE (MASK_PHASE_A_HIGH | MASK_PHASE_B_HIGH \
                                          | MASK_PHASE_C_HIGH)
#define MASK_LOW_SIDE (MASK_PHASE_A_LOW   | MASK_PHASE_B_LOW  \
                                          | MASK_PHASE_C_LOW)
#define MASK_ALL_OUTPUTS (MASK_HIGH_SIDE | MASK_LOW_SIDE)
#define MASK_NO_OUTPUTS  0


#if 0
#define MASK_PHASE_HALL_011 (MASK_PHASE_C_HIGH | MASK_PHASE_C_LOW)
#define  INVCTRL_HALL_011  FTM_INVCTRL_INV0EN_BIT

#define MASK_PHASE_HALL_010 (MASK_PHASE_A_HIGH | MASK_PHASE_A_LOW)
#define   INVCTRL_HALL_010  FTM_INVCTRL_INV2EN_BIT

#define MASK_PHASE_HALL_110 (MASK_PHASE_B_HIGH | MASK_PHASE_B_LOW)
#define   INVCTRL_HALL_110  FTM_INVCTRL_INV2EN_BIT

#define MASK_PHASE_HALL_100 (MASK_PHASE_C_HIGH | MASK_PHASE_C_LOW)
#define   INVCTRL_HALL_100  FTM_INVCTRL_INV1EN_BIT

#define MASK_PHASE_HALL_101 (MASK_PHASE_A_HIGH | MASK_PHASE_A_LOW)
#define   INVCTRL_HALL_101  FTM_INVCTRL_INV1EN_BIT

#define MASK_PHASE_HALL_001 (MASK_PHASE_B_HIGH | MASK_PHASE_B_LOW)
#define   INVCTRL_HALL_001  FTM_INVCTRL_INV0EN_BIT


enum {
    HALL_C_BIT = 1 << 0,
    HALL_B_BIT = 1 << 1,
    HALL_A_BIT = 1 << 2,
};

enum {
    HALL_PHASE_011 = 0,
    HALL_PHASE_010,
    HALL_PHASE_110,
    HALL_PHASE_100,
    HALL_PHASE_101,
    HALL_PHASE_001,
    MAX_HALL_PHASE,

};

typedef struct {
    int ready;
    int hallPosition;
    int commutationMask[MAX_HALL_PHASE];
    int bipolarCompliment[MAX_HALL_PHASE];
} commutator_t;

static commutator_t commutator = {
    .commutationMask = {
        MASK_PHASE_HALL_011,
        MASK_PHASE_HALL_010,
        MASK_PHASE_HALL_110,
        MASK_PHASE_HALL_100,
        MASK_PHASE_HALL_101,
        MASK_PHASE_HALL_001,
    },
    .bipolarCompliment = {
        INVCTRL_HALL_011,
        INVCTRL_HALL_010,
        INVCTRL_HALL_110,
        INVCTRL_HALL_100,
        INVCTRL_HALL_101,
        INVCTRL_HALL_001,
    },
};



static int commutationStepFromHallPos[] = {
    -1,              /* 0b000 is invalid */
    HALL_PHASE_001,  /* 0b001 */
    HALL_PHASE_010,
    HALL_PHASE_011,
    HALL_PHASE_100,
    HALL_PHASE_101,
    HALL_PHASE_110,
};

#endif

static void portAIsr(void)
{
    if ( PORT_PCR(IRQ_PORT, IRQ_PIN)  & PORT_ISF) {
//        printf("TODO: Handle driver interrupts! \n");
        PORT_PCR(IRQ_PORT, IRQ_PIN) |= PORT_ISF;
    }

#if 0
    if ( PORT_PCR(HALL_A_PORT, HALL_A_PIN)  & PORT_ISF) {
        PORT_PCR(HALL_A_PORT, HALL_A_PIN) |= PORT_ISF;
    }
    if ( PORT_PCR(HALL_B_PORT, HALL_B_PIN)  & PORT_ISF) {
        PORT_PCR(HALL_B_PORT, HALL_B_PIN) |= PORT_ISF;
    }
    if ( PORT_PCR(HALL_C_PORT, HALL_C_PIN)  & PORT_ISF) {
        PORT_PCR(HALL_C_PORT, HALL_C_PIN) |= PORT_ISF;
    }

    if (gpioRead(HALL_A_PORT, HALL_A_PIN)) {
        value |= HALL_A_BIT;
    }
    if (gpioRead(HALL_B_PORT, HALL_B_PIN)) {
        value |= HALL_B_BIT;
    }
    if (gpioRead(HALL_C_PORT, HALL_C_PIN)) {
        value |= HALL_C_BIT;
    }
    if (value > 0 && value <= MAX_HALL_PHASE) {
        int idx;
        commutator.hallPosition = value;
        if (0 && commutator.ready) {
            idx = commutationStepFromHallPos[commutator.hallPosition];
            //ftmSetOutputMask(FTM_0, commutator.commutationMask[idx], FALSE);
//            ftmSetInvCtrl(FTM_0, commutator.bipolarCompliment[idx], TRUE);
        }
    } else {
        /* Motor Position Fault! */
        printf("FAULT VALUE %d \n", value);
        gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
        gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
        updateFlags |= UPDATE_HALT;
    }
#endif
   return;
}



enum {
    MOTOR_STATE_OFF,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUN,
};

static scaled_t mechTheta;
static scaled_t theta;
static scaled_t motorOutput     = 0; /* Stopped */
static int32_t  motorState  = MOTOR_STATE_OFF;
#define MOTOR_ALIGN_DRIVE (0.1 * UNITY)

                                                                /* FTM Timing */
static uint32_t msTick;
static uint32_t tickGet()
{
    return msTick;
}
static void __attribute__((optimize("O0"))) taskDelay(uint32_t delayMs) {
    uint32_t wait = tickGet() + delayMs;
    while (tickGet() < wait) {

    }

}

static uint32_t pwmTick;
#define BASE_TIME_BIT (1 << 8)
#define PWM_FREQ 10000
static void callBackFtm0(int index)
{
    static uint32_t updateTick;
    if (index & BASE_TIME_BIT) {
        pwmTick++;
        if (pwmTick > updateTick) {
            msTick++;
            updateTick = pwmTick + (PWM_FREQ / 1000);
        }
    }
}
#define QUAD_CALL_BACK_COUNT 1000
#if 0
static uint32_t encoderCount;
static void callBackFtm2(int index)
{
    encoderCount += index * QUAD_CALL_BACK_COUNT;
}
#endif

static void indexIsr(void)
{
    /* TODO Add index support when we have an encoder w/ index */

    if ( PORT_PCR(ENCODER_INDEX_PORT, ENCODER_INDEX_PIN)  & PORT_ISF) {
        PORT_PCR(ENCODER_INDEX_PORT, ENCODER_INDEX_PIN) |= PORT_ISF;
    }
}


static pidFilter_t pid = {
    .iMin  = -MAX_DRIVE,
    .iMax  =  MAX_DRIVE,
    .iGain = 0.02 * UNITY,
    .pGain = 1 * UNITY,
};

static void filterDrive(void)
{
    scaled_t error;
    int32_t  count  = ftmRead(FTM_2);
    scaled_t elecTheta;
    scaled_t phaseAdvance;
                                                          /* mechanical error */
    mechTheta  = count * 360.0 / COUNTS_PER_360_MECH_DEG * UNITY;
    elecTheta  = mechTheta * ELEC_CYCLES_PER_MECH_CYCLE;
    elecTheta %= 360 * UNITY;
    error = angleSetPoint - mechTheta;

    if (error > 180  * UNITY) {
        error -= 360 * UNITY;
    }
    if (error < -180 * UNITY) {
        error += 360 * UNITY;
    }

    if (error) {

        error /= 100;

        motorOutput = pidFilter(&pid, error);
        if (motorOutput > MAX_DRIVE) {
            motorOutput = MAX_DRIVE;
        } else if (motorOutput < -MAX_DRIVE) {
            motorOutput = -MAX_DRIVE;
        }


        if (motorOutput > 0) {
            theta += 1.8 * UNITY;
        } else if (motorOutput < 0) {
            theta -= 1.8 * UNITY;
        }

        phaseAdvance = theta - elecTheta;
        if (phaseAdvance > 180 * UNITY) {
            phaseAdvance -= 360 * UNITY;
        }
        if (phaseAdvance < -180 * UNITY) {
            phaseAdvance += 360 * UNITY;
        }

        if (phaseAdvance < -MAX_PHASE_ADVANCE) {
            theta = elecTheta - MAX_PHASE_ADVANCE;
        } else if (phaseAdvance > MAX_PHASE_ADVANCE) {
            theta = elecTheta + MAX_PHASE_ADVANCE;
        }

        if (theta < 0) {
            theta += 360 * UNITY;
        } else if (theta > 360 * UNITY) {
            theta -= 360 * UNITY;
        }

        svmOutput = resolveSVM(abs(motorOutput), theta);

        ftmPwmWrite(FTM_0, FTM_CH_1, svmOutput.pwmADuty, FALSE);
        ftmPwmWrite(FTM_0, FTM_CH_3, svmOutput.pwmBDuty, FALSE);
        ftmPwmWrite(FTM_0, FTM_CH_5, svmOutput.pwmCDuty, TRUE);


        gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    }


    return;
}
static void pit0ISR(void)
{
    switch (motorState) {
    case MOTOR_STATE_OFF:
    case MOTOR_STATE_ALIGN:
        /* Handled in main task */
        break;
    case MOTOR_STATE_RUN:
        filterDrive();
        break;
    }

    pitCtrl->pit[PIT_0].flags = 1;
}




/*******************************************************************************
 *
 * initFetPreDriver
 *
 * Reset and initialize the Freescale 33937A FET Pre-driver chip
 *
 *
 ******************************************************************************/
static int initFetPreDriver(int timer, int spiFd)
{
    int retVal = !ERROR;
    uint8_t cmd[8];
    spiWriteRead_t spiTxRx = {
        .out = cmd,
        .in  = cmd,
        .len = 1,
    };

    int i;

    printf("Initializing FET pre driver...\n");

                                       /* Put the chip into reset and disable */
    gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
    gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
    taskDelay(100);
              /* Take chip out of reset and wait for VDD and VLS to stabilize */
    gpioSet(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
    taskDelay(50);


                                              /* Clear interrupt status flags */
    cmd[0] = CLINT0_CMD | 0xf;
    ioctl(spiFd, IO_IOCTL_SPI_WRITE_READ, (int) &spiTxRx);
    cmd[0] = CLINT1_CMD | 0xf;
    ioctl(spiFd, IO_IOCTL_SPI_WRITE_READ, (int) &spiTxRx);

                                            /* Subscribe to interrupt sources */
    cmd[0] = MASK0_CMD | MASK0_OVERTEMP_BIT | MASK0_OVER_CURRENT_BIT
                                            | MASK0_VLS_UNDER_VOLTAGE_BIT ;
    ioctl(spiFd, IO_IOCTL_SPI_WRITE_READ, (int) &spiTxRx);

    cmd[0] = MASK1_CMD | MASK1_FRAMING_ERROR_BIT
                       | MASK1_WRITE_ERROR_BIT | MASK1_RESET_EVENT_BIT;
    ioctl(spiFd, IO_IOCTL_SPI_WRITE_READ, (int) &spiTxRx);

#if 1
    /* Use zero deadtime at pre driver (ftm uses deadtime already) */
    cmd[0] = DEADTIME_CMD;
    printf("read %d \n", ioctl(spiFd, IO_IOCTL_SPI_WRITE_READ, (int) &spiTxRx));

    printf("deadtime returned  %x \n", cmd[0]);
#endif

                           /* Using default deadtime.  Otherwise set it here. */
#if 0
    /* TODO Test this */
    cmd[0] = DEADTIME_CMD | DEADTIME_CALIBRATE_BIT;
    write(spiFd, cmd, 1);
    taskDelay(10);
    read(spiFd, cmd, 1);
    ioctl(spiFd, IO_IOCTL_SPI_SET_OPTS, SPI_OPTS_MASTER | SPI_OPTS_PCS_CONT);
    taskDelay(PERIOD_16_TIMES_LONGER_THAN_DESIRED_DEADTIME);
    ioctl(spiFd, IO_IOCTL_SPI_SET_OPTS, SPI_OPTS_MASTER);
#endif

    /* Setup any special mode settings and lock mode */
    cmd[0] = MODE_CMD | MODE_FULLON_BIT | MODE_DESATURATION_FAULT_BIT;
    //cmd[0] = MODE_CMD | MODE_DESATURATION_FAULT_BIT;
    //| MODE_MODE_LOCK_BIT;
    ioctl(spiFd, IO_IOCTL_SPI_WRITE_READ, (int) &spiTxRx);


            /* Bring up the enable lines (they are tied together in this design)
             * and get ready for some f#cking smoke! :) */
    gpioSet(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);

    cmd[0] = NULL_CMD | 0;
    write(spiFd, cmd, 1);
    taskDelay(50);


    /* Check status for fun */
    for (i = 0; i < MAX_NULL_CMD_REG; i++) {
        cmd[0] = NULL_CMD | i;
        write(spiFd, cmd, 1);
        taskDelay(50);
        read(spiFd, cmd, 1);
//        if (i) {
            if (i==0) {
                cmd[0] &= ~STATUS_REG0_RESET_EVENT;
                if (cmd[0]) {
                    retVal = ERROR;
                }
            }

            printf("FET Pre Amp Status[%d] %x\n", i, cmd[0]);
//        }
    }
//    read(spiFd, cmd, 1);
//    printf("FET Pre Amp Status[%d] %x\n", i, cmd[0]);
    taskDelay(50);

    if (retVal != ERROR) {
        /* Turn on all low sides */
       ftmSetOutputMask(FTM_0, MASK_HIGH_SIDE, TRUE);
       taskDelay(50);

        /* Turn off all low sides */
       ftmSetOutputMask(FTM_0, MASK_ALL_OUTPUTS, TRUE);
       taskDelay(50);

        /* Turn on all high sides */
       ftmSetOutputMask(FTM_0, MASK_LOW_SIDE, TRUE);
       taskDelay(1000);
        /* Turn off all high sides */
       ftmSetOutputMask(FTM_0, MASK_ALL_OUTPUTS, TRUE);

       /* Good to go! */
        printf("FET predriver initialized. \n");

#if 0

        if (gpioRead(HALL_A_PORT, HALL_A_PIN)) {
            value |= HALL_A_BIT;
        }
        if (gpioRead(HALL_B_PORT, HALL_B_PIN)) {
            value |= HALL_B_BIT;
        }
        if (gpioRead(HALL_C_PORT, HALL_C_PIN)) {
            value |= HALL_C_BIT;
        }
        if (value > 0 && value <= MAX_HALL_PHASE) {
            int idx;
            commutator.ready = TRUE;
            commutator.hallPosition = value;
            idx = commutationStepFromHallPos[commutator.hallPosition];
            idx = 2;
//            ftmSetOutputMask(FTM_0, commutator.commutationMask[idx], FALSE);
//            ftmSetInvCtrl(FTM_0, commutator.bipolarCompliment[idx], TRUE);
        } else {
            /* Motor Position Fault! */
            printf("Motor Position Fault! %d \n", value);
            gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
            gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
            updateFlags |= UPDATE_HALT;
        }
#endif


    } else {
        printf("Fault on FET PreDriver! \n");
    //    gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
      //  gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
//        updateFlags |= UPDATE_HALT;
    }

    return retVal;
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
     * Flexbus:  25 MHz
     * Flash:    25 MHz
     */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

    return;


}

int main(void)
{
    int spiFd;
    int fdPot;
    uint32_t updateUserTick  = 0;

    ftmCfg_t ftmPWM = {
        .mode = FTM_MODE_PWM,
        .mod  = 0,
        .initCount     = 0,
        .channels      = { FTM_CH_0, FTM_CH_1,  FTM_CH_2,
                           FTM_CH_3, FTM_CH_4,  FTM_CH_5,
                           FTM_CH_NONE},
        .pwmFreq       = PWM_FREQ,

        .pwmCfgBits    = FTM_PWM_CFG_COMBINED_MODE_CHS_0_1
                       | FTM_PWM_CFG_COMBINED_MODE_CHS_2_3
                       | FTM_PWM_CFG_COMBINED_MODE_CHS_4_5
                       | FTM_PWM_CFG_COMPLEMENTARY_CH_0_1
                       | FTM_PWM_CFG_COMPLEMENTARY_CH_2_3
                       | FTM_PWM_CFG_COMPLEMENTARY_CH_4_5
                       | FTM_PWM_CFG_OUTPUT_MASK
                       | FTM_PWM_CFG_CENTER_ALINGNED
                       ,
        .triggerBits   = FTM_TRIGGER_INIT, /* TODO setup PDB off this trigger
                                            * to run ADC captures of current.
                                            */
        .deadTime      = 1, /* 1/10 uSec */
        .dutyScaled    = { 0.6 * UNITY, 0.6 * UNITY , 0.6 * UNITY ,
                           0.6 * UNITY, 0.6 * UNITY , 0.6 * UNITY},
        .activeLow     = { TRUE, FALSE, TRUE, FALSE, TRUE, FALSE },
    };

    ftmCfg_t ftmCh2QD = {
        .mode = FTM_MODE_QUADRATURE_DECODE,
        .mod  = QUAD_CALL_BACK_COUNT,
        .initCount = 0,
        .quadCfg = FTM_QUAD_CONFIG_FILTER_MED,
    };

    setClock();
    uart_install();
    spi_install();
    adc_install();


    /*
     * Register the standard I/O streams with a particular deivce driver.
     */
    int fd1 = fdevopen(stdout, "uart3", 0, 0);
    ioctl(fd1, IO_IOCTL_UART_BAUD_SET, 115200);
    assert(fd1 != -1);


    gpioConfig(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN,
                                      GPIO_OUTPUT | GPIO_DSE | GPIO_PULLUP);
    gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);

    gpioConfig(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN,
                                       GPIO_OUTPUT | GPIO_DSE | GPIO_PULLUP);
    gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);

    gpioConfig(ENCODER_INDEX_PORT, ENCODER_INDEX_PIN, GPIO_INPUT
                                                   | GPIO_PULLUP);

#if 0
    gpioConfig(HALL_A_PORT, HALL_A_PIN, GPIO_INPUT | GPIO_PULLDOWN);
    gpioConfig(HALL_B_PORT, HALL_B_PIN, GPIO_INPUT | GPIO_PULLDOWN);
    gpioConfig(HALL_C_PORT, HALL_C_PIN, GPIO_INPUT | GPIO_PULLDOWN);
#endif
    gpioConfig(IRQ_PORT, IRQ_PIN, GPIO_INPUT | GPIO_PULLDOWN);

        /* Install irq handler for driver and over current interrupts (PTA27)
         * and phase B,C Hall sensors */
    PORT_PCR(IRQ_PORT, IRQ_PIN) |= PORT_IRQC_INT_RISING_EDGE;

    PORT_PCR(ENCODER_INDEX_PORT, ENCODER_INDEX_PIN)
                                                   |= PORT_IRQC_INT_RISING_EDGE;
#if 0
    PORT_PCR(HALL_A_PORT, HALL_A_PIN) |= PORT_IRQC_INT_EITHER_EDGE;
    PORT_PCR(HALL_B_PORT, HALL_B_PIN) |= PORT_IRQC_INT_EITHER_EDGE;
    PORT_PCR(HALL_C_PORT, HALL_C_PIN) |= PORT_IRQC_INT_EITHER_EDGE;
#endif
    hwInstallISRHandler(ISR_GPIO_A, portAIsr);
    hwInstallISRHandler(ISR_GPIO_B, indexIsr);
#if 0
    hwInstallISRHandler(ISR_GPIO_D, portAIsr);
#endif

                                                                      /* LEDs */
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);
                                                                 /* setup pit */
    pitInit(PIT_0, pit0ISR, 5000);

    hwSetISRPriority(IRQ_PIT0, 0x0F);


                                                                 /* setup ftm */
    ftmInit(FTM_0, callBackFtm0, &ftmPWM);
#if 0
    ftmInit(FTM_2, callBackFtm2, &ftmCh2QD);
#else
    ftmInit(FTM_2, 0, &ftmCh2QD);
#endif

    hwSetISRPriority(IRQ_FTM0, 0x0F);
    hwSetISRPriority(IRQ_FTM2, 0x0F);

    ftmWrite(FTM_2, 1000, 0);
                         /* Install spi into the device table before using it */
    spiFd = open("spi2", 0, 0);
    assert(spiFd != -1);

    ioctl(spiFd, IO_IOCTL_SPI_SET_PORT_PCRS, 0);
    ioctl(spiFd, IO_IOCTL_SPI_SET_BAUD, SPI_BAUDRATE_CLKDIV_32);
    ioctl(spiFd, IO_IOCTL_SPI_SET_SCLK_MODE, SPI_SCLK_MODE_2);
    ioctl(spiFd, IO_IOCTL_SPI_SET_FMSZ, 8);
    ioctl(spiFd, IO_IOCTL_SPI_SET_OPTS, SPI_OPTS_MASTER);
    ioctl(spiFd, IO_IOCTL_SPI_SET_CS, SPI_CS_0);
    ioctl(spiFd, IO_IOCTL_SPI_SET_CS_INACT_STATE, SPI_CS_0_INACT_HIGH);
    ioctl(spiFd, IO_IOCTL_SPI_SET_METHOD, SPI_METHOD_POLLED);


    initFetPreDriver(FTM_0, spiFd);


                                                                 /* ADC Setup */
    fdPot = open("adc0", 0, 0);
    if (fdPot==-1) {
        assert(0);
    }

    ioctl(fdPot, IO_IOCTL_ADC_CALIBRATE, TRUE);


    ioctl(fdPot, IO_IOCTL_ADC_SAMPLE_SIZE_SET, 1);
    ioctl(fdPot, IO_IOCTL_ADC_CALL_BACK_SET, (int)potCallBackHandler);
    ioctl(fdPot, IO_IOCTL_ADC_VREF_SELECT, IO_IOCTL_ADC_VREF_FLAGS_DEFAULT);
    ioctl(fdPot, IO_IOCTL_ADC_TRIGGER_SELECT, IO_IOCTL_ADC_TRIGGER_SELECT_SW);
    ioctl(fdPot, IO_IOCTL_ADC_CONVERSION_CONTINUOUS, TRUE);
    ioctl(fdPot, IO_IOCTL_ADC_CONVERSION_TIME_SELECT,
                 IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_2);
    ioctl(fdPot, IO_IOCTL_ADC_AVERAGE_SELECT, IO_IOCTL_ADC_FLAGS_AVGS_4);
    ioctl(fdPot, IO_IOCTL_ADC_RESOLUTION_SELECT, IO_IOCTL_ADC_RES_FLAGS_12_BIT);
    ioctl(fdPot, IO_IOCTL_ADC_CLOCK_SELECT, IO_IOCTL_ADC_FLAGS_ADICLK_BUS);
    ioctl(fdPot, IO_IOCTL_ADC_DIFFERENTIAL_SET,
                (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A
                  | IO_IOCTL_ADC_DIFF_FLAGS_SINGLE_ENDED));
    ioctl(fdPot, IO_IOCTL_ADC_CHANNEL_SELECT,
                 (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A
                    | (IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DP0
      //              | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DM1
                       & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));

    hwSetISRPriority(IRQ_ADC0, 0x0A);

    while(1){

        switch (motorState) {
        default:
            /* Do nothing */
            break;
        case MOTOR_STATE_OFF:
            /* Turn off motor outputs */
            ftmSetOutputMask(FTM_0, MASK_ALL_OUTPUTS, TRUE);
            taskDelay(1000);
            motorState = MOTOR_STATE_ALIGN;
            break;
        case MOTOR_STATE_ALIGN:
            {
                svm_t svmAlign;
                svmAlign = resolveSVM(MOTOR_ALIGN_DRIVE, 0);
                ftmPwmWrite(FTM_0, FTM_CH_1, svmAlign.pwmADuty, FALSE);
                ftmPwmWrite(FTM_0, FTM_CH_3, svmAlign.pwmBDuty, FALSE);
                ftmPwmWrite(FTM_0, FTM_CH_5, svmAlign.pwmCDuty, FALSE);
                /* Turn on motor outputs */
                ftmSetOutputMask(FTM_0, MASK_NO_OUTPUTS, TRUE);
                taskDelay(250);
                printf("Aligning  %f %f %f \n", svmAlign.pwmADuty / 32768.0,
                       svmAlign.pwmBDuty /32768.0, svmAlign.pwmCDuty / 32768.0);
                printf("Set encoder count %d \n", ftmWrite(FTM_2, 1000, 0));
                motorOutput = MOTOR_ALIGN_DRIVE;
                motorState = MOTOR_STATE_RUN;
            }
            break;
        }

        if (updateFlags & UPDATE_HALT) {
            updateFlags &= ~UPDATE_HALT;
            break;
        }

#if 0
        if (tickGet() > updateStatusTick) {
            updateStatusTick = tickGet() + 5;
#if 0
            if (pwmRampUp) {
                motorOutput += 0.0005 * UNITY;
            } else {
                motorOutput -= 0.0005 * UNITY;
            }
            if (motorOutput > 0.4 * UNITY) {
                motorOutput = 0.4 * UNITY;
                updateStatusTick = tickGet() + 1000;
                pwmRampUp = FALSE;
            } else if (motorOutput < 0.1 * UNITY) {
                motorOutput = 0.1 * UNITY;
                pwmRampUp = TRUE;
                updateStatusTick = tickGet() + 1000;
            }
#endif
        }
#endif

        if (tickGet() > updateUserTick) {
            int count  = ftmRead(FTM_2);

            printf("%d, %3.2f, %3.2f, %3.2f,  0000, "
                   "%d, <<%3.2f>>%3.2f, %3.2f, %3.2f\n",
                   // angleSetPoint / 32768.0,
                    count,
                    angleSetPoint / 32768.0,
                    mechTheta     / 32768.0,
                    theta         / 32768.0,
                    svmOutput.sinLUTIdx,
                    motorOutput / 32768.0,
                    svmOutput.pwmADuty/32768.0,
                    svmOutput.pwmBDuty/32768.0,
                    svmOutput.pwmCDuty/32768.0);

            updateUserTick = tickGet() + 20;
        }
    }

    close(fdPot);
    printf("Application Fatal Error! \n");
    while(1) {};
    return 0;
}
