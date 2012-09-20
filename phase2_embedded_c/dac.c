/*******************************************************************************
*
* dac.c
*
* Low level driver for the Kinetis DAC module.
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

volatile dac_t * const dac0 = ((volatile dac_t * const) DAC0_BASE_ADDR);

void dac0Init(void)
{
    /*
     * Config the DAC0 Clock Gate
     */
    SIM_SCGC2 |= SIM_SCGC2_DAC0_ENABLE;

}


