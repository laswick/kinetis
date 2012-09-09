/*******************************************************************************
*
* tsi.c
*
* Low level driver for the Kinetis TSI module.
*
* API: gsiInit(), tsiRead(), tsiReadRaw()
*
* David Kennedy
* August 27 2012
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

static struct {
    uint32_t portEnable;
    uint32_t port;
    uint32_t pin;
} tsiMap[TSI_COUNT] = {
/* 0*/{SIM_PORTB_ENABLE, PORTB,  0}, {SIM_PORTA_ENABLE, PORTA,  0},
/* 2*/{SIM_PORTA_ENABLE, PORTA,  1}, {SIM_PORTA_ENABLE, PORTA,  2},
/* 4*/{SIM_PORTA_ENABLE, PORTA,  3}, {SIM_PORTA_ENABLE, PORTA,  4},
/* 6*/{SIM_PORTB_ENABLE, PORTB,  1}, {SIM_PORTB_ENABLE, PORTB,  2},
/* 8*/{SIM_PORTB_ENABLE, PORTB,  3}, {SIM_PORTB_ENABLE, PORTB, 16},
/*10*/{SIM_PORTB_ENABLE, PORTB, 17}, {SIM_PORTB_ENABLE, PORTB, 18},
/*12*/{SIM_PORTB_ENABLE, PORTB, 19}, {SIM_PORTC_ENABLE, PORTC,  0},
/*14*/{SIM_PORTC_ENABLE, PORTC,  1}, {SIM_PORTC_ENABLE, PORTC,  2},
};
int32_t tsiInit(const tsiConfig_t *cfg)
{
    int pin;

    SIM_SCGC5 |= SIM_TSI_ENABLE;

    for (pin=0; pin < TSI_COUNT; pin ++) {
        if (cfg->pinEnable & (1 << pin)) {
            SIM_SCGC5 |= tsiMap[pin].portEnable;
            PORT_PCR(tsiMap[pin].port, tsiMap[pin].pin) = PORT_IRQC_DISABLED |
                                                                PORT_MUX_ANALOG;
            TSI0_THRESHOLD[pin] = (1 << TSI_THRESHOLD_LTHH_SHIFT) |
                                           (0xFFFE << TSI_THRESHOLD_HTHH_SHIFT);
        }
    }

    TSI0_PEN = cfg->pinEnable;
    TSI0_SCANC = cfg->scanc;
    TSI0_GENCS = ((1-1) << TSI_GENCS_NSCN_SHIFT) |
                          (cfg->prescale << TSI_GENCS_PS_SHIFT) | TSI_GENCS_STM;

    TSI0_GENCS |= TSI_GENCS_TSIEN;

    return 1;
}

uint32_t tsiRead(const tsiConfig_t *cfg)
{
    int pin;
    uint32_t result = 0;

    for (pin=0; pin < TSI_COUNT; pin++) {
        if (cfg->pinEnable & (1 << pin)) {
            if (TSI0_CNTR[pin] > cfg->threshold[pin]) {
                result |= (1 << pin);
            }
        }
    }
    return result;
}

uint32_t tsiReadRaw(uint32_t pin)
{
    assert((pin < TSI_COUNT));

    return TSI0_CNTR[pin];
}
