/*******************************************************************************
*
* demoBldc.c
*
* James McAnanama
*
* This demo drives the a BLDC motor using the TWR-MC-LV3PH board.
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

#define UNITY 32768


static int updateFlags;
enum {
    UPDATE_NONE         = 0,
    UPDATE_HALT         = 1 << 1,
};


#define FET_DRIVER_RESET_PORT PORTE
#define FET_DRIVER_RESET_PIN  27
#define FET_DRIVER_ENABLE_PORT PORTE
#define FET_DRIVER_ENABLE_PIN  28
#define IRQ_PORT PORTA
#define IRQ_PIN 27

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



#define MASK_PHASE_HALL_011_CCW (  MASK_PHASE_A_HIGH | MASK_PHASE_B_LOW \
                                | MASK_PHASE_C_HIGH | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_010_CCW (  MASK_PHASE_A_HIGH | MASK_PHASE_A_LOW \
                                | MASK_PHASE_B_LOW  | MASK_PHASE_C_HIGH)


#define MASK_PHASE_HALL_110_CCW (  MASK_PHASE_A_LOW  | MASK_PHASE_B_HIGH \
                                | MASK_PHASE_B_LOW  | MASK_PHASE_C_HIGH)

#define MASK_PHASE_HALL_100_CCW ( MASK_PHASE_A_LOW   | MASK_PHASE_B_HIGH \
                               | MASK_PHASE_C_HIGH | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_101_CCW (  MASK_PHASE_A_HIGH | MASK_PHASE_A_LOW \
                                | MASK_PHASE_B_HIGH | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_001_CCW (  MASK_PHASE_A_HIGH | MASK_PHASE_B_HIGH \
                                | MASK_PHASE_B_LOW  | MASK_PHASE_C_LOW)




#define MASK_PHASE_HALL_101_CW (  MASK_PHASE_A_HIGH | MASK_PHASE_A_LOW \
                                 | MASK_PHASE_B_LOW  | MASK_PHASE_C_HIGH)

#define MASK_PHASE_HALL_100_CW ( MASK_PHASE_A_HIGH  | MASK_PHASE_B_LOW \
                                | MASK_PHASE_C_HIGH  | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_110_CW (  MASK_PHASE_A_HIGH | MASK_PHASE_B_HIGH \
                                 | MASK_PHASE_B_LOW  | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_010_CW (  MASK_PHASE_A_HIGH | MASK_PHASE_A_LOW \
                                 | MASK_PHASE_B_HIGH | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_011_CW (  MASK_PHASE_A_LOW  | MASK_PHASE_B_HIGH \
                                 | MASK_PHASE_C_HIGH | MASK_PHASE_C_LOW)

#define MASK_PHASE_HALL_001_CW (  MASK_PHASE_A_LOW  | MASK_PHASE_B_HIGH \
                                 | MASK_PHASE_B_LOW  | MASK_PHASE_C_HIGH)

enum {
    HALL_C_BIT = 1 << 0,
    HALL_B_BIT = 1 << 1,
    HALL_A_BIT = 1 << 2,
};

enum {
    HALL_PHASE_011_CCW = 0,
    HALL_PHASE_010_CCW,
    HALL_PHASE_110_CCW,
    HALL_PHASE_100_CCW,
    HALL_PHASE_101_CCW,
    HALL_PHASE_001_CCW,
    MAX_HALL_PHASE,

    HALL_PHASE_110_CW = 0,
    HALL_PHASE_010_CW,
    HALL_PHASE_011_CW,
    HALL_PHASE_001_CW,
    HALL_PHASE_101_CW,
    HALL_PHASE_100_CW,

};

enum {
    COMMUTATE_CCW,
    COMMUTATE_CW,
};


typedef struct {
    int ready;
    int direction;
    int hallPosition;
    int commutationMaskCCW[MAX_HALL_PHASE];
    int commutationMaskCW[MAX_HALL_PHASE];
} commutator_t;

static commutator_t commutator = {
    .commutationMaskCCW = {
        MASK_PHASE_HALL_011_CCW,
        MASK_PHASE_HALL_010_CCW,
        MASK_PHASE_HALL_110_CCW,
        MASK_PHASE_HALL_100_CCW,
        MASK_PHASE_HALL_101_CCW,
        MASK_PHASE_HALL_001_CCW,
    },
    .commutationMaskCW = {
        MASK_PHASE_HALL_110_CW,
        MASK_PHASE_HALL_010_CW,
        MASK_PHASE_HALL_011_CW,
        MASK_PHASE_HALL_001_CW,
        MASK_PHASE_HALL_101_CW,
        MASK_PHASE_HALL_100_CW,
    },
};



static int commutationStepCCWFromHallPos[] = {
    -1,              /* 0b000 is invalid */
    HALL_PHASE_001_CCW,  /* 0b001 */
    HALL_PHASE_010_CCW,
    HALL_PHASE_011_CCW,
    HALL_PHASE_100_CCW,
    HALL_PHASE_101_CCW,
    HALL_PHASE_110_CCW,
};

static int commutationStepCWFromHallPos[] = {
    -1,              /* 0b000 is invalid */
    HALL_PHASE_001_CW,  /* 0b001 */
    HALL_PHASE_010_CW,
    HALL_PHASE_011_CW,
    HALL_PHASE_100_CW,
    HALL_PHASE_101_CW,
    HALL_PHASE_110_CW,
};



static void portAIsr(void)
{
    int value = 0;
    if ( PORT_PCR(IRQ_PORT, IRQ_PIN)  & PORT_ISF) {
//        printf("TODO: Handle driver interrupts! \n");
        PORT_PCR(IRQ_PORT, IRQ_PIN) |= PORT_ISF;
    }
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
        if (commutator.ready) {
            if (commutator.direction == COMMUTATE_CCW) {
                idx = commutationStepCCWFromHallPos[commutator.hallPosition];
                ftmSetOutputMask(FTM_0, commutator.commutationMaskCCW[idx]);
            } else {
                idx = commutationStepCWFromHallPos[commutator.hallPosition];
                ftmSetOutputMask(FTM_0, commutator.commutationMaskCW[idx]);
            }
        }
    } else {
        /* Motor Position Fault! */
        printf("FAULT VALUE %d \n", value);
        gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
        gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
        updateFlags |= UPDATE_HALT;
    }
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
     * Flexbus:  25 MHz
     * Flash:    25 MHz
     */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);

    return;


}

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
#define PWM_FREQ 16000
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

#if 0
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
    //cmd[0] = MODE_CMD | MODE_FULLON_BIT | MODE_DESATURATION_FAULT_BIT;
    cmd[0] = MODE_CMD | MODE_DESATURATION_FAULT_BIT;
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
        int value = 0;
        /* Turn on all low sides */
       ftmSetOutputMask(FTM_0, MASK_HIGH_SIDE);
       taskDelay(1000);
        /* Turn off all low sides */
       ftmSetOutputMask(FTM_0, MASK_ALL_OUTPUTS);
       taskDelay(50);

        /* Turn on all high sides */
       ftmSetOutputMask(FTM_0, MASK_LOW_SIDE);
       taskDelay(1000);
        /* Turn off all high sides */
       ftmSetOutputMask(FTM_0, MASK_ALL_OUTPUTS);

       /* Good to go! */
        printf("FET predriver initialized. \n");

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
            if (commutator.direction == COMMUTATE_CCW) {
                idx = commutationStepCCWFromHallPos[commutator.hallPosition];
                ftmSetOutputMask(FTM_0, commutator.commutationMaskCCW[idx]);
            } else {
                idx = commutationStepCWFromHallPos[commutator.hallPosition];
                ftmSetOutputMask(FTM_0, commutator.commutationMaskCW[idx]);
            }
        } else {
            /* Motor Position Fault! */
            printf("Motor Position Fault! %d \n", value);
            gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
            gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
            updateFlags |= UPDATE_HALT;
        }


    } else {
        printf("Fault on FET PreDriver! \n");
    //    gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);
      //  gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);
//        updateFlags |= UPDATE_HALT;
    }

    return retVal;
}


int main(void)
{
    int spiFd;
    uint32_t updateStatusTick = 0;
#if 0
    int i;
    int tests[NUM_SPI_METHODS] = { FALSE };
#endif


    ftmCfg_t ftmPWM = {
        .mode = FTM_MODE_PWM,
        .mod  = 64,
        .initCount     = 0,
        .channels      = { FTM_CH_0, FTM_CH_1,  FTM_CH_2,
                           FTM_CH_3, FTM_CH_4,  FTM_CH_5,
                           FTM_CH_NONE},
        .pwmFreq       = PWM_FREQ,

        .pwmCfgBits    = FTM_PWM_CFG_COMBINED_MODE_CHS_0_1
                       | FTM_PWM_CFG_COMBINED_MODE_CHS_2_3
                       | FTM_PWM_CFG_COMBINED_MODE_CHS_4_5
#if 0
                       | FTM_PWM_CFG_COMPLEMENTARY_CH_0_1
                       | FTM_PWM_CFG_COMPLEMENTARY_CH_2_3
                       | FTM_PWM_CFG_COMPLEMENTARY_CH_4_5
#endif
                       | FTM_PWM_CFG_OUTPUT_MASK,
        .triggerBits   = FTM_TRIGGER_INIT, /* TODO setup PDB off this trigger
                                            * to run ADC captures of current.
                                            */
    //    .deadTime      = 1, /* 1 uSec */
#if 1
        .dutyScaled    = { 0.25 * UNITY, 0.75 * UNITY , 0.25 * UNITY ,
                           0.75 * UNITY, 0.25 * UNITY , 0.75 * UNITY},
#endif
        .activeLow     = { TRUE, FALSE, TRUE, FALSE, TRUE, FALSE },
    };

    setClock();

    uart_install();
    /*
     * Register the standard I/O streams with a particular deivce driver.
     */
    int fd1 = fdevopen(stdout, "uart3", 0, 0);
    ioctl(fd1, IO_IOCTL_UART_BAUD_SET, 115200);
    assert(fd1 != -1);

    printf("\r\n");
    printf("\r\n = SPI TEST APPLICATION = \r\n");

    gpioConfig(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN,
                                      GPIO_OUTPUT | GPIO_DSE | GPIO_PULLUP);
    gpioClear(FET_DRIVER_ENABLE_PORT, FET_DRIVER_ENABLE_PIN);

    gpioConfig(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN,
                                       GPIO_OUTPUT | GPIO_DSE | GPIO_PULLUP);
    gpioClear(FET_DRIVER_RESET_PORT, FET_DRIVER_RESET_PIN);

    gpioConfig(HALL_A_PORT, HALL_A_PIN, GPIO_INPUT | GPIO_PULLDOWN);
    gpioConfig(HALL_B_PORT, HALL_B_PIN, GPIO_INPUT | GPIO_PULLDOWN);
    gpioConfig(HALL_C_PORT, HALL_C_PIN, GPIO_INPUT | GPIO_PULLDOWN);
    gpioConfig(IRQ_PORT, IRQ_PIN, GPIO_INPUT | GPIO_PULLDOWN);

        /* Install irq handler for driver and over current interrupts (PTA27)
         * and phase B,C Hall sensors */
    PORT_PCR(IRQ_PORT, IRQ_PIN) |= PORT_IRQC_INT_RISING_EDGE;
    PORT_PCR(HALL_A_PORT, HALL_A_PIN) |= PORT_IRQC_INT_EITHER_EDGE;
    PORT_PCR(HALL_B_PORT, HALL_B_PIN) |= PORT_IRQC_INT_EITHER_EDGE;
    PORT_PCR(HALL_C_PORT, HALL_C_PIN) |= PORT_IRQC_INT_EITHER_EDGE;
    hwInstallISRHandler(ISR_GPIO_A, portAIsr);
    hwInstallISRHandler(ISR_GPIO_D, portAIsr);





    /* setup ftm */
    ftmInit(FTM_0, callBackFtm0, &ftmPWM);

    /* Install spi into the device table before using it */
    spi_install();
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

    int pwmDuty = 0.2 * UNITY;
    int pwmRampUp = TRUE;
    while(1){

        if (updateFlags & UPDATE_HALT) {
            updateFlags &= ~UPDATE_HALT;
            break;
        }

        if (tickGet() > updateStatusTick) {
            updateStatusTick = tickGet() + 5;
            if (pwmRampUp) {
                pwmDuty += 0.004 * UNITY;
            } else {
                pwmDuty -= 0.004 * UNITY;
            }
            if (pwmDuty > 0.9999 * UNITY) {
                updateStatusTick = tickGet() + 2000;
                pwmRampUp = FALSE;
            } else if (pwmDuty < 0.4 * UNITY) {
                pwmRampUp = TRUE;
                if (commutator.direction == COMMUTATE_CCW) {
                    commutator.direction = COMMUTATE_CW;
                } else {
                    commutator.direction = COMMUTATE_CCW;
                }
            }
            ftmPwmWrite(FTM_0, FTM_CH_0, pwmDuty);
            ftmPwmWrite(FTM_0, FTM_CH_3, pwmDuty);
            ftmPwmWrite(FTM_0, FTM_CH_5, pwmDuty);
        }

    }
    printf("Application Fatal Error! \n");
    while(1) {};
    return 0;
}
