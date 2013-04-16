#include <stdio.h>
//#include <stdlib.h>
#include "util.h"


                                                   /* Space Vector Modulation */
#define MOTOR_ZERO_OUTPUT (0.5 * UNITY)
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
#define COUNTS_PER_360_ELEC_DEG  (1000 / ELEC_CYCLES_PER_MECH_CYCLE)
#define COUNTS_PER_60_ELEC_DEG   33
#define MAX_INCREMENTAL_DRIVE    0x7fffffff //(3 * 60.0 / COUNTS_PER_60_ELEC_DEG * UNITY)
static const int32_t sinLUT[] = {
        0,      1040,   2078,   3115,   4148,
        5177,   6201,   7219,   8230,   9232,
        10225,  11207,  12179,  13138,  14084,
        15015,  15932,  16832,  17716,  18581,
        19428,  20256,  21063,  21849,  22613,
        23354,  24071,  24764,  25433,  26076,
        26692,  27282,  27844,  28377,
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
#define SCALE(value, shift) (((value) + (1 << (shift)) / 2) >> (shift))
static int svmDuty(scaled_t duty[3], scaled_t relativeAngle[2],
                                      scaled_t output, int *idxLog)
{
    int i;
    scaled_t dutySum = 0;
    scaled_t pwmOutput;
    int idx;
    for (i = 0; i < 2; i++) {
        /* sin(theta) */
        idx = divS(relativeAngle[i] * COUNTS_PER_60_ELEC_DEG, SVM_60_DEGREES);
        idx = SCALE(idx, 15);
#if 0
        printf("%d idx %d, angle %3.3f, ratio %3.2f, cat %d \n",
                i, idx, relativeAngle[i]/32768.0,
                relativeAngle[i]/32768.0 / 60.0 * COUNTS_PER_60_ELEC_DEG,
                relativeAngle[i] * COUNTS_PER_60_ELEC_DEG);
#endif
        idxLog[i] = idx;
        duty[i] = sinLUT[idx];//relativeAngle[i] * COUNTS_PER_60_ELEC_DEG
                                //            / SVM_60_DEGREES];
    }
        /* pseudo normalized pwm output to match desired drive across the svm */
//    pwmOutput = output * UNITY / (duty[0] + duty[1]);
    pwmOutput = output;

    for (i = 0; i < 2; i++) {
        /* Vector i duty cycle = output * sin(theta) */
        duty[i] = duty[i] * pwmOutput / UNITY;
        dutySum += duty[i];
    }
    /* Store half the remainder of the %pwm period  for use across the channels */
    duty[2] = (UNITY - dutySum) / 2;

    return idx;
}
static int idxLog[2];
static svm_t resolveSVM(scaled_t output, scaled_t angle)
{
    svm_t svm;
    scaled_t relativeAngle[2];
    scaled_t duty[3]; /* [duty_1, duty_2, half duty_0] */

    angle %= 360 * UNITY;

    if (angle < SVM_VECTOR_60) {
        relativeAngle[1] = angle;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output, idxLog);
        svm.pwmADuty = duty[0] + duty[1] + duty[2];
        svm.pwmBDuty =           duty[1] + duty[2];
        svm.pwmCDuty =                     duty[2];
    } else if (angle < SVM_VECTOR_120) {

        relativeAngle[1] = angle - SVM_VECTOR_60;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output, idxLog);
        svm.pwmADuty = duty[0]           + duty[2];
        svm.pwmBDuty = duty[0] + duty[1] + duty[2];
        svm.pwmCDuty =                     duty[2];
    } else if (angle < SVM_VECTOR_180) {

        relativeAngle[1] = angle - SVM_VECTOR_120;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output, idxLog);
        svm.pwmADuty =                     duty[2];
        svm.pwmBDuty = duty[0] + duty[1] + duty[2];
        svm.pwmCDuty =           duty[1] + duty[2];
    } else if (angle < SVM_VECTOR_240) {

        relativeAngle[1] = angle - SVM_VECTOR_180;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output, idxLog);
        svm.pwmADuty =                     duty[2];
        svm.pwmBDuty = duty[0]           + duty[2];
        svm.pwmCDuty = duty[0] + duty[1] + duty[2];
    } else if (angle < SVM_VECTOR_300) {

        relativeAngle[1] = angle - SVM_VECTOR_240;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output, idxLog);
        svm.pwmADuty =           duty[1] + duty[2];
        svm.pwmBDuty =                     duty[2];
        svm.pwmCDuty = duty[0] + duty[1] + duty[2];
    } else {

        relativeAngle[1] = angle - SVM_VECTOR_300;
        relativeAngle[0] = SVM_60_DEGREES - relativeAngle[1];
        svm.sinLUTIdx = svmDuty(duty, relativeAngle, output, idxLog);
        svm.pwmADuty = duty[0] + duty[1] + duty[2];
        svm.pwmBDuty =                     duty[2];
        svm.pwmCDuty = duty[0]           + duty[2];
    }

    return svm;
}

static uint32_t count;
static scaled_t angleSetPoint;
static scaled_t mechTheta;
static scaled_t theta;
static scaled_t incTheta;
static scaled_t motorOutput = 0.1 * UNITY;

static pidFilter_t pid = {
    .iMin  = -MAX_INCREMENTAL_DRIVE,
    .iMax  =  MAX_INCREMENTAL_DRIVE,
    .iGain = 0.01 * UNITY,
    .pGain = 0.2 * UNITY,
};


#define FTM_2 0
int32_t ftmRead(int FTM) {

    int32_t retVal;

    count = (mechTheta / 360.0 * 1000) / UNITY;
    retVal = count;


    return retVal;
}

static void filterDrive(void)
{
    /* mechanical error */
    scaled_t error = angleSetPoint - mechTheta;

    if (error > 180  * UNITY) {
        error -= 360 * UNITY;
    }
    if (error < -180 * UNITY) {
        error += 360 * UNITY;
    }


   /* Convert to electrical angle error */
    error *= ELEC_CYCLES_PER_MECH_CYCLE;


    theta = ftmRead(FTM_2) * 360.0 / COUNTS_PER_360_MECH_DEG * UNITY;
    theta *= 5;
    //theta += 1.8 * UNITY;


    incTheta = pidFilter(&pid, error);
#if 1
    if (incTheta > 3 * 1.8 * UNITY) {
        incTheta = 3 * 1.8 * UNITY;
    }
    if (incTheta < 3 * -1.8 * UNITY) {
        incTheta = 3 * -1.8 * UNITY;
    }
#endif

    theta += incTheta;
    theta %= 360 * UNITY;



    svmOutput = resolveSVM(motorOutput, theta);
    mechTheta += incTheta / 5;



    return;
}

int main(void)
{
    angleSetPoint  = 179 * UNITY;
    int idx = 0;

#if 0
    while (abs(angleSetPoint - mechTheta) > 0.0001 * UNITY) {
        filterDrive();
        printf("%d, %3.2f, %3.2f, %3.2f,  0000, "
                "%d, %3.2f, %3.2f, %3.2f, %d\n",
                // angleSetPoint / 32768.0,
                count,
                angleSetPoint / 32768.0,
                mechTheta     / 32768.0,
                theta         / 32768.0,
                svmOutput.sinLUTIdx,
                svmOutput.pwmADuty/32768.0,
                svmOutput.pwmBDuty/32768.0,
                svmOutput.pwmCDuty/32768.0,
                idx++);
        delay();

    }
#else

    for (idx = 0 * UNITY; idx < 360 * UNITY; idx += 1.8221 * UNITY) {
        svmOutput = resolveSVM(UNITY, idx);
        printf("%3.2f, %d, %d, %3.4f, %3.4f, %3.4f \n",
                idx / 32768.0,
                idxLog[0], idxLog[1],
                svmOutput.pwmADuty/32768.0,
                svmOutput.pwmBDuty/32768.0,
                svmOutput.pwmCDuty/32768.0
                );


    }



#endif

    return 1;
}
