/*******************************************************************************
*
* gpioDemo2.c
*
* GPIO demo.
*
* Rob Laswick
* June 19 2012
*
*******************************************************************************/
//TODO put all the assert stuff in hardware.c and hardware.h.
//TODO put all the global gpio APIs in gpio.c and extern them in hardware.h.

/*
 * standard headers
 * processors headers
 * operating system headers
 * custom headers
 */

#include "kinetis.h"

//#include "hardware.h"

#include "globalDefs.h"


#if defined(assert)
#undef assert
#endif

#define assert(cond) ((cond)? (void) 0 : assert_(__FILE__, __LINE__))
//#define assert(cond) ( {if (!cond) assert_(__FILE__, __LINE__);} )

void assert_(const char *file, const int line)
{
    /*
     * This routine has no impact on the running software,
     * however it's a good idea to set a breakpoint on assert_.
     *
     * When this breakpoint hits, gdb will automatically display
     * the functin arguments (files and line).
     */
}


#define N_LED_ORANGE_PORT PORTA
#define N_LED_ORANGE_BIT  11

#define N_LED_YELLOW_PORT PORTA
#define N_LED_YELLOW_BIT  28

#define N_LED_GREEN_PORT  PORTA
#define N_LED_GREEN_BIT   29

#define N_LED_BLUE_PORT   PORTA
#define N_LED_BLUE_BIT    10

#define LEDS_MASK  (1 << N_LED_ORANGE_BIT) | (1 << N_LED_YELLOW_BIT) \
                 | (1 << N_LED_GREEN_BIT)  | (1 << N_LED_BLUE_BIT)



#if 0 /* typical */
typedef struct {
} gpioConfig_t;
void gpioInit(gpioConfig_t *cfg)
#endif

enum {                                                        /* GPIO Options */
    GPIO_INPUT     = BIT_0,                                          /* Input */
    GPIO_OUTPUT    = BIT_1,                                         /* Output */
    GPIO_DSE       = BIT_2,                          /* Drive Strength Enable */
    GPIO_ODE       = BIT_3,                              /* Open Drain Enable */
    GPIO_HIGH      = BIT_4,                  /* Default HIGH when ODE is TRUE */
    GPIO_LOW       = BIT_5,                   /* Default LOW when ODE is TRUE */
    GPIO_PFE       = BIT_6,                          /* Passive Filter Enable */
    GPIO_PULLUP    = BIT_7,                                  /* Pullup Enable */
    GPIO_PULLDOWN  = BIT_8,                                /* Pulldown Enable */
};

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

void gpioSet(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    gpioPort->psor = 1 << pin;
}

void gpioClear(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    gpioPort->pcor = 1 << pin;
}

void gpioToggle(uint32_t port, uint32_t pin)
{
    volatile gpioPort_t *gpioPort = gpioPortGet(port);
    gpioPort->ptor = 1 << pin;
}

unsigned gpioPortWrite(uint32_t port, uint32_t mask, uint32_t value)
{
}

unsigned gpioPortRead(uint32_t port)
{
}

unsigned gpioRead(uint32_t port, uint32_t pin)
{
}

static void delay(void)
{
    volatile uint32_t time = 0x0007ffff;
    while (time)
        --time;
}

int main(void)
{
    assert(0);                          /* test ... worth while demonstrating */
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_BIT,  GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_BIT,   GPIO_OUTPUT | GPIO_LOW);

    for (;;) {
        delay();
        gpioClear(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT);
        delay();
        gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT);
        delay();
        gpioClear(N_LED_GREEN_PORT, N_LED_GREEN_BIT);
        delay();
        gpioClear(N_LED_BLUE_PORT, N_LED_BLUE_BIT);

        delay();
        gpioSet(N_LED_ORANGE_PORT, N_LED_ORANGE_BIT);
        delay();
        gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_BIT);
        delay();
        gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_BIT);
        delay();
        gpioSet(N_LED_BLUE_PORT, N_LED_BLUE_BIT);
    }

    return 0;
}
