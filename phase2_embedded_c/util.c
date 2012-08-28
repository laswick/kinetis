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
#include "globalDefs.h"

void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}


