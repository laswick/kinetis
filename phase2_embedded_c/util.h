/*******************************************************************************
*
* util.h
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
#include "globalDefs.h"
#if !defined(UTIL_H)
#define UTIL_H

extern void delay(void);

#define SCALE_TO_INT(scaled, shift) (((scaled) + (1 << (shift)) / 2) >> (shift))
extern scaled_t addS(scaled_t a, scaled_t b);
extern scaled_t subS(scaled_t a, scaled_t b);
extern scaled_t mulS(scaled_t a, scaled_t b);
extern scaled_t divS(scaled_t a, scaled_t b);


typedef struct {
    scaled_t iState;  /* Integrator State */
    scaled_t iMin;    /* Integrator limits */
    scaled_t iMax;
    scaled_t iGain;   /* Integrator gain (< 1) */
    scaled_t pGain;   /* Proportional gain (> 1) */
    scaled_t dState;  /* Differentiator state */
    scaled_t dpGain;  /* Differentiator filter gain = 1 - pole */
    scaled_t dGain;   /* Derivative gain * (1 - pole) */
} pidFilter_t;

extern scaled_t pidFilter(pidFilter_t *pid, scaled_t input);

#endif
