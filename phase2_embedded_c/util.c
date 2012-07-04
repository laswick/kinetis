/*******************************************************************************
*
* util.c
*
* $Id: $
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


