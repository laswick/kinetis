/*******************************************************************************
*
* hardware.h
*
*******************************************************************************/
#include "globalDefs.h"

/* ASSERT *********************************************************************/

#if defined(assert)
#undef assert
#endif

extern void assert_(const char *file, const int line);

#define assert(cond) ((cond)? (void) 0 : assert_(__FILE__, __LINE__))

/* GPIO ***********************************************************************/

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

void gpioConfig(uint32_t port, uint32_t pin, uint32_t opt);
void gpioSet(uint32_t port, uint32_t pin);
void gpioClear(uint32_t port, uint32_t pin);
void gpioToggle(uint32_t port, uint32_t pin);
uint32_t gpioPortWrite(uint32_t port, uint32_t mask, uint32_t value);
uint32_t gpioPortRead(uint32_t port);
uint32_t gpioRead(uint32_t port, uint32_t pin);

/* EXAMPLE ********************************************************************/

typedef struct {
} featureConfig_t;                                        /* i.e. spiConfig_t */

int32_t featureInit(featureConfig_t *cfg);
int32_t featureWrite(uint8_t *buffer, int32_t len);
int32_t featureRead(uint8_t *buffer, int32_t len);

/******************************************************************************/

/* UART    ********************************************************************/

enum {                                                  /* UART baud  Options */
    UART_BAUD_1200,
    UART_BAUD_2400,
    UART_BAUD_4800,
    UART_BAUD_9600,
    UART_BAUD_14400,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200,
};


int32_t featureInit(featureConfig_t *cfg);
int32_t featureWrite(uint8_t *buffer, int32_t len);
int32_t featureRead(uint8_t *buffer, int32_t len);

/******************************************************************************/



























