/*******************************************************************************
*
* pit.c
*
* Low level driver for the Kinetis PIT module.
*
* API:
*
* Daryl Hillman
* Sept 2012
*
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

volatile pitCtrl_t * const pitCtrl
                         = ((volatile pitCtrl_t * const) PIT_CTRL_BASE);

void pitInit(int timer, void *isr, uint32_t initCount)
{
    assert(timer >= 0);
    assert(timer < MAX_PIT);

    SIM_SCGC6 |= SIM_SCGC6_PIT_ENABLE;

    hwInstallISRHandler(ISR_PIT0 + timer, isr);

    pitCtrl->mcr = 1;
    pitCtrl->pit[timer].loadVal = initCount;
    pitCtrl->pit[timer].ctrl = 0x3;

}


