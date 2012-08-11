/*******************************************************************************
*
* gpio.c
*
* Low level driver for the Kinetis GPIO module.
*
* API: gpioConfig(), gpioSet(), gpioClear, gpioToggle(), gpioRead(),
*      gpioPortWrite(), gpioPortRead().
*
* Rob Laswick
* June 19 2012
*
*******************************************************************************/
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

/*******************************************************************************
*
* gpioPortGet
*
* This helper function determines the appropriate gpio module base address
* based on the argument "port".
*
* RETURNS: The corresponding GPIO_BASE_ADDR.
*
*******************************************************************************/
static volatile gpioPort_t *gpioPortGet(uint32_t port)
{
    uint32_t addr;

    switch (port) {
    case PORTA: addr = GPIOA_BASE_ADDR; break;
    case PORTB: addr = GPIOB_BASE_ADDR; break;
    case PORTC: addr = GPIOC_BASE_ADDR; break;
    case PORTD: addr = GPIOD_BASE_ADDR; break;
    case PORTE: addr = GPIOE_BASE_ADDR; break;
    default:
        assert(0);
        return 0;
    }

    return ((volatile gpioPort_t *) addr);
}

/*******************************************************************************
*
* gpioConfig
*
* This routine configures the provided port pin. All available config
* options are in the GPIO section in hardware.h.
*
* RETURNS: Nothing
*
*******************************************************************************/
void gpioConfig(uint32_t port, uint32_t pin, uint32_t opt)
{
    /*
     * Ensure pin number is legal
     */

    assert((pin < 32));

    /*
     * Config the SIM Clock Gate
     */

    switch (port) {
    case PORTA: SIM_SCGC5 |= SIM_PORTA_ENABLE; break;
    case PORTB: SIM_SCGC5 |= SIM_PORTB_ENABLE; break;
    case PORTC: SIM_SCGC5 |= SIM_PORTC_ENABLE; break;
    case PORTD: SIM_SCGC5 |= SIM_PORTD_ENABLE; break;
    case PORTE: SIM_SCGC5 |= SIM_PORTE_ENABLE; break;
    default:
        assert(0);
        return;
    }

    /*
     * Configure the Port Controller
     */

    uint32_t portCtrlBits = PORT_MUX_GPIO;

    if (opt & GPIO_OUTPUT) {
        if (opt & GPIO_ODE) {
            portCtrlBits |= PORT_ODE;
        } else {
            if (opt & GPIO_DSE)
                portCtrlBits |= PORT_DSE;
        }

    } else if (opt & GPIO_INPUT) {
        if (opt & GPIO_PFE)
            portCtrlBits |= PORT_PFE;

    } else {
        assert(0);
        return;
    }

    if (opt & GPIO_PULLUP)
        portCtrlBits |= PORT_PULLUP_ENABLE;
    else if (opt & GPIO_PULLDOWN)
        portCtrlBits |= PORT_PULLDOWN_ENABLE;

    PORT_PCR(port, pin) = portCtrlBits;

    /*
     * Configure the GPIO Controller
     */

    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    uint32_t pinBit = 1 << pin;

    if (opt & GPIO_OUTPUT) {
        gpioPort->pddr |= pinBit;
        if (opt & GPIO_HIGH)
            gpioPort->psor |= pinBit;
        else if (opt & GPIO_LOW)
            gpioPort->pcor |= pinBit;
    }
}

/*******************************************************************************
* gpioSet
*******************************************************************************/
void gpioSet(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    gpioPort->psor = 1 << pin;
}

/*******************************************************************************
* gpioClear
*******************************************************************************/
void gpioClear(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    gpioPort->pcor = 1 << pin;
}

/*******************************************************************************
* gpioToggle
*******************************************************************************/
void gpioToggle(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    gpioPort->ptor = 1 << pin;
}

/*******************************************************************************
*
* gpioPortWrite
*
* This routine...
*
* RETURNS:
*
*******************************************************************************/
void gpioPortWrite(uint32_t port, uint32_t mask, uint32_t value)
{
}

/*******************************************************************************
*
* gpioPortRead
*
* This routine...
*
* RETURNS:
*
*******************************************************************************/
uint32_t gpioPortRead(uint32_t port)
{
    return 0;
}

/*******************************************************************************
*
* gpioRead
*
* This routine...
*
* RETURNS:
*
*******************************************************************************/
uint32_t gpioRead(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    return ((gpioPort->pdir & (1 << pin)) ? TRUE : FALSE);
}

