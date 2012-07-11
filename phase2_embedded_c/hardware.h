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

extern void gpioConfig(uint32_t port, uint32_t pin, uint32_t opt);
extern void gpioSet(uint32_t port, uint32_t pin);
extern void gpioClear(uint32_t port, uint32_t pin);
extern void gpioToggle(uint32_t port, uint32_t pin);
extern void gpioPortWrite(uint32_t port, uint32_t mask, uint32_t value);
extern uint32_t gpioPortRead(uint32_t port);
extern uint32_t gpioRead(uint32_t port, uint32_t pin);

/* EXAMPLE ********************************************************************/

typedef struct {
} featureConfig_t;                                        /* i.e. spiConfig_t */

extern int32_t featureInit(const featureConfig_t *cfg);
extern int32_t featureWrite(uint8_t *buffer, int32_t len);
extern int32_t featureRead(uint8_t *buffer, int32_t len);

/*******************************************************************************
*
* CCA Hardware Defines
*
*******************************************************************************/
#if defined(FREESCALE_K60N512_TOWER_HW)

/* LEDS ***********************************************************************/

#define N_LED_ORANGE_PORT PORTA
#define N_LED_ORANGE_PIN  11

#define N_LED_YELLOW_PORT PORTA
#define N_LED_YELLOW_PIN  28

#define N_LED_GREEN_PORT  PORTA
#define N_LED_GREEN_PIN   29

#define N_LED_BLUE_PORT   PORTA
#define N_LED_BLUE_PIN    10

/* UART ***********************************************************************/

#define MAX_UARTS     5

#define UART4_PORT    PORTE
#define UART4_RX_PIN  25
#define UART4_TX_PIN  24

#define UART3_PORT    PORTC
#define UART3_RX_PIN  16
#define UART3_TX_PIN  17

typedef struct {
    uint32_t uart;
    int32_t  systemClockHz;
    int32_t  busClockHz;
    int32_t  baud;
    uint16_t responseWaitTime;
/* TODO    uint8_t  loopback; */
} uartIF_t;

extern int32_t uartInit(uartIF_t *cfg);
extern void    uartFree(uartIF_t *cfg);
extern int32_t uartPrint(uartIF_t *cfg, char *string);
extern int32_t uartWrite(uartIF_t *cfg, uint8_t *buffer, int32_t len);
extern int32_t uartRead(uartIF_t *cfg, uint8_t *buffer, int32_t len);
extern void    setStdout(uartIF_t *uartIF);

/******************************************************************************/

#else
#error Undefined Hardware Platform
#endif



























