/*******************************************************************************
*
* util.c
*
* James McAnanama
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include "util.h"
#include <stdio.h>
#include "globalDefs.h"

void delay(void)
{
    volatile uint32_t time = 0x00fffff;
    while (time)
        --time;
}

#define FIN_BIG_INT  0x7fffffff


scaled_t addS(scaled_t a, scaled_t b)
{
    scaled_t retVal = a + b;
    if (a < 0 && b < 0) {
        if (retVal > 0 || retVal < -FIN_BIG_INT) {
            retVal = -FIN_BIG_INT;
        }
    } else if (a > 0 && b > 0) {
        if (retVal < 0) {
            retVal = FIN_BIG_INT;
        }
    }

    return retVal;
}


scaled_t subS(scaled_t a, scaled_t b)
{
    scaled_t retVal = a - b;

    if (a < 0 && b > 0) {
        if (retVal > 0 || retVal < -FIN_BIG_INT) {
            retVal = -FIN_BIG_INT;
        }
    } else if (a > 0 && b < 0) {
        if (retVal < 0) {
            retVal = FIN_BIG_INT;
        }
    }

    return retVal;
}


scaled_t mulS(scaled_t a, scaled_t b)
{
    long long retVal = (long long)a * (long long)b;

    retVal += 0.5 * UNITY;
    retVal >>= 15;

    if (retVal > FIN_BIG_INT) {
        retVal = FIN_BIG_INT;
    }

    if (retVal < -FIN_BIG_INT) {
        retVal = -FIN_BIG_INT;
    }

    return (scaled_t) retVal;
}

scaled_t divS(scaled_t a, scaled_t b)
{
    long long retVal = (long long) a << 15;

    if (b == 0) {
        if (a < 0) {
            retVal = -FIN_BIG_INT;
        } else {
            retVal = FIN_BIG_INT;
        }
    } else {

        retVal /= (long long) b;
        if (retVal > FIN_BIG_INT) {
            retVal = FIN_BIG_INT;
        }
        if (retVal < -FIN_BIG_INT) {
            retVal = -FIN_BIG_INT;
        }
    }
    return retVal;
}


/******************************************************************************
 *  pidFilter
 *
 *  pid filter based on the fine work of Tim Wescott,
 *  "Applied Control Theory for Embedded Systems"  Newnes 2006.
 *
 *
 ******************************************************************************/
scaled_t pidFilter(pidFilter_t *pid, scaled_t input)
{
    scaled_t retVal;
    scaled_t dTemp;
    scaled_t pTerm = mulS(input, pid->pGain);

    retVal = pTerm;

    if (pid->iGain) {
        pid->iState = addS(pid->iState, mulS(pid->iGain, input));
        retVal = addS(retVal, pid->iState);
        if (retVal > pid->iMax) {
            pid->iState = subS(pid->iMax, pTerm);
            retVal = pid->iMax;
        } else if (retVal < pid->iMin) {
            pid->iState = subS(pid->iMin, pTerm);
            retVal = pid->iMin;
        }
    }

    if (pid->dpGain) {
        dTemp = subS(input, pid->dState);
        pid->dState += mulS(pid->dpGain, dTemp);
    }


    retVal = addS(retVal, mulS(dTemp, pid->dGain));

    return retVal;
}

