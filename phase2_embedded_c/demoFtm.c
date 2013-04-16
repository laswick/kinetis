/*******************************************************************************
*
* demoFtm.c
*
* Simple ftm demo.
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
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"
#include "util.h"


static uint32_t msTick;
static uint32_t tickGet()
{
    return msTick;
}
static void taskDelay(uint32_t delayMs) {
    uint32_t wait = tickGet() + delayMs;

    while (tickGet() < wait) {
        wait = wait + 1 -550 + 275 * 2 - 1;
    }

}



static int updateFlags;
enum {
    UPDATE_DRIVE_MODE = 1 << 0,
};


static int32_t driveMode;
enum {
    DRIVE_MODE_OFF,
    DRIVE_MODE_OPEN_LOOP,
    DRIVE_MODE_POSITION, /* Distance setpoint and open loop rate input */

    MAX_DRIVE_MODE,
    DRIVE_MODE_SKID_STEER, /* Speed setpoint to each motor */
};
#define DRIVE_MODE_STRINGS "OFF", "OPEN_LOOP", "SKID STEER", "POSITION"
static const char * const driveModeStrings[] = { DRIVE_MODE_STRINGS };

#define WHEEL_CIRCUM 134 /* Lego NXT wheel 134mm */
#define MAX_SPEED (0.19 * 32768)
#define MAX_DIST  250
#define GO_BUTTON_PORT PORTC
#define GO_BUTTON_PIN  0


#define MOTOR_STDBY_PORT PORTA
#define MOTOR_STDBY_PIN  4


#define MOTOR_AIN_1_PORT PORTB
#define MOTOR_AIN_1_PIN  2
#define MOTOR_AIN_2_PORT PORTB
#define MOTOR_AIN_2_PIN  3

#define MOTOR_BIN_1_PORT PORTB
#define MOTOR_BIN_1_PIN  16
#define MOTOR_BIN_2_PORT PORTB
#define MOTOR_BIN_2_PIN  17

#define GO_BUTTON_PIN  0

static void latchGo(void)
{
    static uint32_t debounceTick;

    if (tickGet() > debounceTick) {
        driveMode++;
        if (driveMode >= MAX_DRIVE_MODE) {
            driveMode = DRIVE_MODE_OFF;
        }
        updateFlags |= UPDATE_DRIVE_MODE;
        debounceTick = tickGet() + 1000;
    }
    PORT_PCR(GO_BUTTON_PORT, GO_BUTTON_PIN) |= PORT_ISF;
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
     * Bus:      25 MHz
     * Flexbus:  25 MHz
     * Flash:    25 MHz
     */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_4, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

    return;


}

static int mSpeed[2];
static int mPos[2] = {0,0};
static int skidDuty[2];
static int skidError[2];
static int posDuty[2];
static int posError[2];
static int32_t motorIdx[2];
static int32_t pots[2];
static int32_t setPoint[2];
static uint32_t pwmTick;

#define BASE_TIME_BIT (1 << 8)
static void callBackFtm0(int index)
{
    static uint32_t updateTick;
    if (index & BASE_TIME_BIT) {
        pwmTick++;
        if (pwmTick > updateTick) {
            msTick++;
            updateTick = pwmTick + 100;
        }
    }
}

static void callBackFtm1(int index)
{
    motorIdx[0] += index;
}

static void callBackFtm2(int index)
{
    motorIdx[1] += index;
}

static void driveOff(void)
{
    ftmPwmWrite(FTM_0, FTM_CH_0, 0, FALSE);
    ftmPwmWrite(FTM_0, FTM_CH_1, 0, TRUE);
    gpioClear(MOTOR_STDBY_PORT, MOTOR_STDBY_PIN);


}

static void driveOpenLoop(void)
{
    int dutyM1 = abs(setPoint[0]);
    int dutyM2 = abs(setPoint[0]);


    gpioClear(MOTOR_STDBY_PORT, MOTOR_STDBY_PIN);
    if (setPoint[0] >= 0) {
        gpioSet(  MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN);
        gpioClear(MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN);

        gpioClear(MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN);
        gpioSet(  MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN);
    } else {
        gpioClear(MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN);
        gpioSet(  MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN);

        gpioSet(  MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN);
        gpioClear(MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN);
    }

    if (setPoint[1] >= 0) {
        dutyM2 -= setPoint[1];
    } else {
        dutyM1 += setPoint[1];
    }

    gpioSet(MOTOR_STDBY_PORT, MOTOR_STDBY_PIN);

    ftmPwmWrite(FTM_0, FTM_CH_0, dutyM1, FALSE);
    ftmPwmWrite(FTM_0, FTM_CH_1, dutyM2, TRUE);
}

static void driveSkidSteer(void)
{
    int i;
    int speed[2] = {(setPoint[0] * MAX_SPEED) / 32768,
                    (setPoint[1] * MAX_SPEED) / 32768};

    if (speed[0] >= 0) {
        gpioSet(MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN);
        gpioClear(MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN);
    } else {
        gpioClear(MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN);
        gpioSet(MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN);
    }

    if (speed[1] >= 0) {
        gpioClear(MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN);
        gpioSet(  MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN);
    } else {
        gpioSet(  MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN);
        gpioClear(MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN);
    }

    gpioSet(MOTOR_STDBY_PORT, MOTOR_STDBY_PIN);
    for (i = 0; i < 2; i++) {
        skidDuty[i] += 0.1 * (abs(speed[i]) - abs(mSpeed[i]));
        skidError[i] = abs(speed[i]) - abs(mSpeed[i]);
        if (skidDuty[i] < 0) {
            skidDuty[i] = 0;
        }
        if (skidDuty[i] > 32768) {
            skidDuty[i] = 32768;
        }
    }
    ftmPwmWrite(FTM_0, FTM_CH_0, skidDuty[0], FALSE);
    ftmPwmWrite(FTM_0, FTM_CH_1, skidDuty[1], TRUE);
}


static void drivePosition(void)
{
    int i;
    int pos   = (setPoint[0] * MAX_DIST)  / 32768;

    for (i = 0; i < 2; i++) {
        int mPosNow = mPos[i];

        if ((pos - mPosNow) >= 0) {
            if (i == 0) {
                gpioSet(  MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN);
                gpioClear(MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN);
            } else {

                gpioClear(MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN);
                gpioSet(MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN);
            }
        } else {
            if (i == 0) {
                gpioClear(MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN);
                gpioSet(  MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN);
            } else {

                gpioSet(  MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN);
                gpioClear(MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN);
            }
        }

        gpioSet(MOTOR_STDBY_PORT, MOTOR_STDBY_PIN);
        posDuty[i]  =  100 * abs(pos -mPosNow);
        posError[i] = pos - mPosNow;
        if (posDuty[i] < 0) {
            posDuty[i] = 0;
        }
        if (posDuty[i] > 32768) {
            posDuty[i] = 32768;
        }
    }
    ftmPwmWrite(FTM_0, FTM_CH_0, posDuty[0], FALSE);
    ftmPwmWrite(FTM_0, FTM_CH_1, posDuty[1], TRUE);
}

int main(void)
{
    int i;
    int quit  = FALSE;

    uint32_t updatePotTick   = 0;
    uint32_t updateSpeedTick = 0;
    uint32_t updateUserTick  = 0;
    uint32_t updateMotorTick = 0;
    uint32_t updateLedTick   = 0;

    char string[256];

    ftmCfg_t ftmPWM = {
        .mode = FTM_MODE_PWM,
        .mod  = 64,
        .initCount     = 0,
        .channels      = {FTM_CH_0, FTM_CH_1, FTM_CH_NONE}, /* Terminate with FTM_CH_NONE */
        .pwmFreq       = 100000,
        .dutyScaled    = {0.1 * 32768, 0.5 * 32768},
    };

    ftmCfg_t ftmCh1QD = {
        .mode = FTM_MODE_QUADRATURE_DECODE,
        .mod  = 720,
        .initCount = 0,
    };
    ftmCfg_t ftmCh2QD = {
        .mode = FTM_MODE_QUADRATURE_DECODE,
        .mod  = 720,
        .initCount = 0,
    };

    setClock();


    /* Install uart and adc into the device table before using them */
    uart_install();
    adc_install();

    /*
     * Register the standard I/O streams with a particular deivce driver.
     */

    int fd1 = fdevopen(stdout, "uart3", 0, 0);
    ioctl(fd1, IO_IOCTL_UART_BAUD_SET, 115200);
    assert(fd1 != -1);

    /*
     * Configure ADC.
     */

    int fdPot = open("adc1", 0, 0);
    if (fdPot==-1) {
        assert(0);
    }

    ioctl(fdPot, IO_IOCTL_ADC_CALIBRATE, TRUE);


    ioctl(fdPot, IO_IOCTL_ADC_TRIGGER_SELECT, IO_IOCTL_ADC_TRIGGER_SELECT_SW);
    ioctl(fdPot, IO_IOCTL_ADC_CONVERSION_CONTINUOUS, TRUE);
    ioctl(fdPot, IO_IOCTL_ADC_CONVERSION_TIME_SELECT,
                 IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_20);
    ioctl(fdPot, IO_IOCTL_ADC_AVERAGE_SELECT, IO_IOCTL_ADC_FLAGS_AVGS_32);
    ioctl(fdPot, IO_IOCTL_ADC_RESOLUTION_SELECT, IO_IOCTL_ADC_RES_FLAGS_8_BIT);
    ioctl(fdPot, IO_IOCTL_ADC_CLOCK_SELECT, IO_IOCTL_ADC_FLAGS_ADICLK_BUS);
    ioctl(fdPot, IO_IOCTL_ADC_DIFFERENTIAL_SET,
                (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A
                  | IO_IOCTL_ADC_DIFF_FLAGS_SINGLE_ENDED));
    ioctl(fdPot, IO_IOCTL_ADC_CHANNEL_SELECT,
                 (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A
                    | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DP1
                       & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));


    /*
     * Configure FTM.
     * FTM0: PWM Driver - I'm using two channels to drive two motors.
     *       Also using the counter overflow as my ms clock source.
     * FMT1: Quadrature decoder (Note - this doesn't work properly when the
     *       TWR-SER board is installed on the tower as the pins are shared with
     *       some of the ethernet nonsense.
     * FTM2: Quadrature decoder
     *
     */
    ftmInit(FTM_2, callBackFtm2, &ftmCh2QD);
    ftmInit(FTM_1, callBackFtm1, &ftmCh1QD);
    ftmInit(FTM_0, callBackFtm0, &ftmPWM);

    ftmWrite(FTM_1, 720, 0);
    ftmWrite(FTM_2, 720, 0);

    /*
     * GPIO:
     * Leds used to indicated drive mode.
     * Discrete IOs used for:
     *  -Direction and stby of motor driver.
     *  -Drive mode change input.
     *
     */
    gpioConfig(MOTOR_STDBY_PORT, MOTOR_STDBY_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(MOTOR_AIN_1_PORT, MOTOR_AIN_1_PIN, GPIO_OUTPUT
                                                | GPIO_LOW | GPIO_DSE);
    gpioConfig(MOTOR_AIN_2_PORT, MOTOR_AIN_2_PIN, GPIO_OUTPUT
                                                | GPIO_LOW | GPIO_DSE);
    gpioConfig(MOTOR_BIN_1_PORT, MOTOR_BIN_1_PIN, GPIO_OUTPUT
                                                | GPIO_LOW | GPIO_DSE);
    gpioConfig(MOTOR_BIN_2_PORT, MOTOR_BIN_2_PIN, GPIO_OUTPUT
                                                | GPIO_LOW | GPIO_DSE);

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT, N_LED_GREEN_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT, N_LED_BLUE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(GO_BUTTON_PORT, GO_BUTTON_PIN, GPIO_INPUT
                                            | GPIO_PULLUP | GPIO_PFE);
    PORT_PCR(GO_BUTTON_PORT, GO_BUTTON_PIN) |= PORT_IRQC_INT_FALLING_EDGE;


    /* Drive mode handler */
    hwInstallISRHandler(ISR_GPIO_C, latchGo);
#if 0
    /* Draw the ascii animation. Showing use of ms timer. */
    for (i=0 ; i < 512; i++) {
        printf("\r\n");
    }
    printf("The start of something good. %s %s\r", "<0==0>", "[OO++0\\");
    fflush(stdout);
    taskDelay(2000);
    for (i=0; i < 60; i++) {
        if (i < 30) {
        printf("The start of something good. %*s %*s\r",
                                                         i, "<0==0>",
                                                         i, "[OO++0\\");
        } else {
        printf("The start of something good. %*s%*s%*s\r", i, "<0==0",
                                                        2 * (i - 28), "=->",
                                                 i - 2 * (i - 30), "[OO++0\\");
        }
        fflush(stdout);
        taskDelay(75);
    }
    printf("The start of something good. %*s%*s\r\n", i, "<0==0",
                                  2 * (i-30) + 15, "* *BOOM! @@*..___...     ");
    taskDelay(500);
#endif
    updateFlags = UPDATE_DRIVE_MODE;
    while (!quit) {

        if (updateFlags & UPDATE_DRIVE_MODE) {
            updateFlags &= ~UPDATE_DRIVE_MODE;
            for (i=0 ; i < 512; i++) {
                printf("\r\n");
            }

            printf("\n======== DRIVE MODE IS NOW %s ======== \n",
                                                   driveModeStrings[driveMode]);
            ftmWrite(FTM_1, 720, 0);
            ftmWrite(FTM_2, 720, 0);
            motorIdx[0] = motorIdx[1] = 0;
         }

        if (tickGet() > updateLedTick) {
            switch (driveMode) {
            case DRIVE_MODE_OFF:
                gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
                gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
                break;
            case DRIVE_MODE_OPEN_LOOP:
                gpioToggle(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

                gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
                gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
                break;
            case DRIVE_MODE_SKID_STEER:
                gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);

                gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
                gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
                break;
            case DRIVE_MODE_POSITION:
                gpioToggle(N_LED_GREEN_PORT, N_LED_GREEN_PIN);

                gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
                gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
                break;
            default:
                assert(0);
                gpioToggle(N_LED_BLUE_PORT, N_LED_BLUE_PIN);

                gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
                gpioToggle(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
                break;
            }

            updateLedTick = tickGet() + 250;
        }


        if (tickGet() > updateSpeedTick) {

            int i;
            int32_t count[2] = { ftmRead(FTM_1), ftmRead(FTM_2) };
            static uint32_t prevTick;

            for (i = 0; i < 2; i++) {
                int pos;
                int motorIndex = motorIdx[i];
                if  (motorIndex < 0) {
                    pos = WHEEL_CIRCUM * (motorIndex + 1)
                        + WHEEL_CIRCUM * (-720 + count[i]) /720;
                } else {
                    pos = WHEEL_CIRCUM * motorIndex
                        + WHEEL_CIRCUM * count[i] / 720;
                }
                mSpeed[i] = (pos - mPos[i]) * 32768
                                / (int)(tickGet() - prevTick);
                mPos[i] = pos;
            }
            prevTick = tickGet();
            updateSpeedTick = tickGet() + 50;
        }

        if (tickGet() > updatePotTick) {
            static int32_t duty1;
            static int32_t duty2;
            ioctl(fdPot, IO_IOCTL_ADC_CHANNEL_SELECT,
                    (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A
                     | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DP1
                         & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));
            read(fdPot, &pots[0], 1);

            ioctl(fdPot, IO_IOCTL_ADC_CHANNEL_SELECT,
                    (IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A
                     | (IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE16
                         & IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK)));
            read(fdPot, &pots[1], 1);

                         /* If I told you, they wouldn't be magic would they? */
            duty1 = (int)(0.95 * duty1 + 0.05 * (pots[0] - 106) * 32768 / 106);
            duty2 = (int)(0.95 * duty2 + 0.05 * (pots[1] - 106) * 32768 / 106);

            setPoint[0] = duty1; /* Hee hee hee... duty... */
            setPoint[1] = duty2;

            updatePotTick = tickGet() + 10;
        }

        if (tickGet() > updateUserTick) {
            switch (driveMode) {
            case DRIVE_MODE_OFF:
                sprintf(string, "                              \t");
                for (i = 0; i < 2; i++) {
                    sprintf(string,
                           "%s ||Motor %d: , Pos %d cm, Speed %3.4f m/s||\t",
                            string, i + 1, mPos[i] / 10, mSpeed[i] / 32768.0);
                }
                break;
            case DRIVE_MODE_OPEN_LOOP:
                sprintf(string, "Pots: FWD/REV %3.2f, L/R %3.2f\t",
                        setPoint[0] / 32768.0, setPoint[1]/32768.0);
                for (i = 0; i < 2; i++) {
                    sprintf(string,
                               "%s ||Motor %d: Pos %d cm, Speed %3.4f m/s|| \t",
                              string, i + 1, mPos[i] / 10, mSpeed[i] / 32768.0);
                }
                break;
            case DRIVE_MODE_SKID_STEER:
#if 1
                sprintf(string, "Setpoint: Speed %3.2f\t",
                        (setPoint[1] * MAX_SPEED)/ 32768 /32768.0);
                sprintf(string,
                        "%s ||Motor: Duty %f, Speed %3.4f m/s error %3.2f|| \t",
                                string, skidDuty[1] / 32768.0,
                                mSpeed[1] / 32768.0, skidError[1]/32768.0);

#else
                sprintf(string, "Setpoint: Speed 1 %3.2f, Speed 2 %3.2f\t",
                        (setPoint[0] * MAX_SPEED) / 32768 / 32768.0,
                        (setPoint[1] * MAX_SPEED)/ 32768 /32768.0);
                for (i = 0; i < 2; i++) {
                    sprintf(string,
                               "%s ||Motor %d: Duty %f, Speed %3.4f m/s error %3.2f|| \t",
                                string, i + 1, skidDuty[i] / 32768.0,
                                mSpeed[i] / 32768.0, skidError[i]/32768.0);
                }
#endif

                break;
            case DRIVE_MODE_POSITION:

                sprintf(string, "Setpoint: Pos 1 %d cm            \t",
                       (int) (setPoint[0] * MAX_DIST) / 327680);
#if 1
                sprintf(string,
                               "%s ||Motor: Duty %f, Pos %d cm error %d cm|| \t",
                                string, posDuty[1] / 32768.0,
                                mPos[1]/10, posError[1]/10);

#else
                for (i = 0; i < 2; i++) {
                    sprintf(string,
                               "%s ||Motor %d: Duty %f, Pos %d cm error %d cm|| \t",
                                string, i + 1, posDuty[i] / 32768.0,
                                mPos[i]/10, posError[i]/10);
                }
#endif
                break;
            }
            printf("%s\r", string);
            fflush(stdout);
            updateUserTick = tickGet() + 1000;
        }

        if (tickGet() > updateMotorTick) {
            switch (driveMode) {
            case DRIVE_MODE_OFF:
                driveOff();
                break;
            case DRIVE_MODE_OPEN_LOOP:
                driveOpenLoop();
                break;
            case DRIVE_MODE_SKID_STEER:
                driveSkidSteer();
                break;
            case DRIVE_MODE_POSITION:
                drivePosition();
                break;
            default:
                assert(0);
                break;
            }
            updateMotorTick = tickGet() + 100;
        }


    }
    close(fd1);
    close(fdPot);
    return 0;
}
