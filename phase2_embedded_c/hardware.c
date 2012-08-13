/*******************************************************************************
*
* hardware.c
*
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"

#include "globalDefs.h"
#include "hardware.h"

/*******************************************************************************
*
* assert_
*
* This routine has no impact on the running software, however it's a good idea
* to set a breakpoint on assert_.
*
* When this breakpoint hits, gdb will automatically display the function
* arguments (files and line).
*
* RETURNS: Nothing
*
*******************************************************************************/
void assert_(const char *file, const int line) { }

/******************************************************************************
 *
 * hwInstallISR
 *
 * Installs an ISR Handler into a ram copy of the vector table. The vector
 * table is relocated to RAM as part of startcode.s. This routine also enables
 * the interrupt in the NVIC module
 *
 * RETURNS: Nothing
 *
 *****************************************************************************/
extern uint32_t _vector_ram_start;
void hwInstallISRHandler(uint32_t isr, void *isrHandler)
{
    volatile uint32_t *vectorTbl = (uint32_t *)&_vector_ram_start;
    volatile uint32_t *nvicICPR  = (uint32_t *)NVIC_ICPR_BASE_ADDR;
    volatile uint32_t *nvicISER  = (uint32_t *)NVIC_ISER_BASE_ADDR;

    assert(isr < MAX_ISR);

    hwInterruptsDisable();

    vectorTbl[isr]  = (uint32_t)isrHandler;
    vectorTbl[isr] |= 0x1;  /* Ensure the thumb mode bit is set */

    if (isr > ISR_SYSTICK) {
        int irq = isr - (ISR_SYSTICK + 1); /* Convert to an irq number by
                                          subtracting off the 16 core vectors */
        nvicICPR += (irq / 32);
        nvicISER += (irq / 32);

        *nvicICPR = irq % 32;    /* Clear pending interrupt */
        *nvicISER = irq % 32;    /* Enable interrupt */
    }

    hwInterruptsEnable();

    /* Some core ISRs are not enabled by default. Enable as registered */
    switch (isr) {
    case ISR_BUS_FAULT:
        ARM_SHCSR |= ARM_SHCSR_BUSFAULTENA;
        break;
    }
}
