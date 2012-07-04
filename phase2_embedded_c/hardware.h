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
void gpioPortWrite(uint32_t port, uint32_t mask, uint32_t value);
uint32_t gpioPortRead(uint32_t port);
uint32_t gpioRead(uint32_t port, uint32_t pin);

/* EXAMPLE ********************************************************************/

typedef struct {
} featureConfig_t;                                        /* i.e. spiConfig_t */

int32_t featureInit(featureConfig_t *cfg);
int32_t featureWrite(uint8_t *buffer, int32_t len);
int32_t featureRead(uint8_t *buffer, int32_t len);

/*******************************************************************************
*
* CCA Hardware Defines
*
*******************************************************************************/
#if PLATFORM == FREESCALE_K60N512_TOWER

/* LEDS ***********************************************************************/

#define N_LED_ORANGE_PORT PORTA
#define N_LED_ORANGE_PIN  11

#define N_LED_YELLOW_PORT PORTA
#define N_LED_YELLOW_PIN  28

#define N_LED_GREEN_PORT  PORTA
#define N_LED_GREEN_PIN   29

#define N_LED_BLUE_PORT   PORTA
#define N_LED_BLUE_PIN    10

#endif



/* UART    ********************************************************************/
#if PLATFORM == FREESCALE_K60N512_TOWER
#define UART4_PORT    PORTE
#define UART4_RX_PIN  25
#define UART4_TX_PIN  24

#define UART3_PORT    PORTC
#define UART3_RX_PIN  16
#define UART3_TX_PIN  17

#else
#error "UNKNOWN UART CONFIGURATION"
#endif

typedef struct {
    uint32_t uart;
    int32_t  systemClockHz;
    int32_t  busClockHz;
    int32_t  baud;
    uint16_t responseWaitTime;
/* TODO    uint8_t  loopback; */
} uartIF_t;


int32_t uartInit(uartIF_t *cfg);
void    uartFree(uartIF_t *cfg);
int32_t uartWrite(uartIF_t *cfg, uint8_t *buffer, int32_t len);
int32_t  uartRead(uartIF_t *cfg, uint8_t *buffer, int32_t len);

/******************************************************************************/



























