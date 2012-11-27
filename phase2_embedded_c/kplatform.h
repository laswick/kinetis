/*******************************************************************************
*
* kplatform.h
*
* Steve Holford
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#if !defined(KPLATFORM_H)
#define KPLATFORM_H

#include "globalDefs.h"

/* Set architecture flags based on the defined target board */

#if defined(FREESCALE_K60N512_TOWER_HW) /* Integer only, 512K Flash Tower Board */

#define K60N512

#elif defined(FREESCALE_K60F120_TOWER_HW) /* Floating point, 1024K Flash Tower Board */

#define K60F120

#elif defined(FREESCALE_K70F120_TOWER_HW) /* FP, 1024K Flash, 1Gig DDR2 Tower Board */

#define K70F120

#else 

#error Undefined Hardware Platform

#endif

#endif /* !defined(KPLATFORM_H) */

