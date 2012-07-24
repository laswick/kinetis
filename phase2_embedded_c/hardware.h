/*******************************************************************************
*
* hardware.h
*
*******************************************************************************/
#include <stdlib.h>
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

/* SPI  ***********************************************************************/

/*********** Control the SPI module in can be done in 2 ways. ****************
 * Normal Mode: Makes the interface simple and does not require the user
 *              to understand the register set for the SPI module. The
 *              disadvantage being that some of the more obscure features are
 *              not controllable.
 * Raw Mode: All the features are available to the user, in fact the user
 *           writes to a 'shadow' register of the SPI moudle. All features
 *           of the SPI module are available, however intimite knowledge of
 *           the SPI register set is required.
 *****************************************************************************/

/* Mode select for the spi module */
typedef enum {
    SPI_SCLK_MODE_0 = 0, /* CPOL = 0, CPHA = 0, ***DEFAULT*** */
    SPI_SCLK_MODE_1,     /* CPOL = 0, CPHA = 1                */
    SPI_SCLK_MODE_2,     /* CPOL = 1, CPHA = 0                */
    SPI_SCLK_MODE_3,     /* CPOL = 1, CPHA = 1                */
    NUMSPI_SCLK_MODES
} spiSclkMode_t;

/* Mask for all the spi module options */
typedef enum {
    SPI_OPTS_MASTER         = (1<<0), /* Module is SPI Master */
    SPI_OPTS_LSB_FIRST      = (1<<1), /* LSB of frame is transferred first */
    SPI_OPTS_CONT_SCK_EN    = (1<<2), /* Continuously run the SCK */
    SPI_OPTS_TX_FIFO_DSBL   = (1<<3), /* Disable Tx FIFO */
    SPI_OPTS_RX_FIFO_DSBL   = (1<<4), /* Disable Rx FIFO */
    SPI_OPTS_RX_FIFO_OVR_EN = (1<<5), /* Receive FIFO Overflow Overwrite */
    SPI_OPTS_PCS_CONT       = (1<<6), /* Keep pcs pins asserted between transfers */
} spiOptions_t;

/* Mask for chip selects to assert when writing / reading */
typedef enum {
    SPI_CS_0 = (1<<0),
    SPI_CS_1 = (1<<1),
    SPI_CS_2 = (1<<2),
    SPI_CS_3 = (1<<3),
    SPI_CS_4 = (1<<4),
    SPI_CS_5 = (1<<5),
} spiChipSelect_t;

/* Mask for the inactive state of the chip select, 0 = INACT_LOW */
typedef enum {
    SPI_CS_0_INACT_HIGH = (1<<0),
    SPI_CS_1_INACT_HIGH = (1<<1),
    SPI_CS_2_INACT_HIGH = (1<<2),
    SPI_CS_3_INACT_HIGH = (1<<3),
    SPI_CS_4_INACT_HIGH = (1<<4),
    SPI_CS_5_INACT_HIGH = (1<<5),
} spiCSInactState_t;

/* Baud rate selection for the SPI */
typedef enum {
    SPI_BAUDRATE_CLKDIV_4,
    SPI_BAUDRATE_CLKDIV_8,
    SPI_BAUDRATE_CLKDIV_12,
    SPI_BAUDRATE_CLKDIV_16,
    SPI_BAUDRATE_CLKDIV_32,
    SPI_BAUDRATE_CLKDIV_64,
    SPI_BAUDRATE_CLKDIV_128,
    SPI_BAUDRATE_CLKDIV_256,
    SPI_BAUDRATE_CLKDIV_512,
    SPI_BAUDRATE_CLKDIV_1024,
    SPI_BAUDRATE_CLKDIV_2048,
    SPI_BAUDRATE_CLKDIV_4096,
    SPI_BAUDRATE_CLKDIV_8192,
    SPI_BAUDRATE_CLKDIV_16384,
    SPI_BAUDRATE_CLKDIV_32768,
    SPI_BAUDRATE_CLKDIV_65536,
    NUM_SPI_BAUDRATES,
} spiBaudRate_t;

#define SPI_FMSZ_MAX 16
#define SPI_FMSZ_MIN 3

/* Select which SPI module to use */
typedef enum {
    SPI_MODULE_0,
    SPI_MODULE_1,
    SPI_MODULE_2,
    NUM_SPI_MODULES
} spiModule_t;

/* Select the control mode */
typedef enum {
    SPI_CTRL_TYPE_NORMAL,
    SPI_CTRL_TYPE_RAW,
    NUM_SPI_CTRLS
} spiCtrlType_t;

/* RAW SPI Control */
/* See kinetis.h for contants to use for this struct  */
typedef struct spiRaw_s {
    spiMcr_t   mcr;
    spiCtar_t  ctar0;
    spiCtar_t  ctar1;
    spiRser_t  rser;
    spiPushr_t pushr;
} spiRaw_t;

/* Normal SPI Control */
typedef struct spiNml_s {
    spiSclkMode_t       sclkMode;
    spiBaudRate_t       baudRate;
    spiOptions_t        options;
    spiChipSelect_t     chipSelect;
    spiCSInactState_t   csInactState;
    unsigned            frameSize;   /* See SPI_FMSZ_MAX/MIN for limits */
} spiNml_t;

typedef struct spiIF_s {
    spiCtrlType_t ctrlType;
    spiModule_t   module;
    unsigned      modAddr;

    union {
        spiRaw_t  raw;
        spiNml_t  nml;
    }ctrl;

} spiIF_t;

void spiOpen(spiIF_t *spi);
void spiClose(spiIF_t *spi);
void spiWrite(spiIF_t *spi, void *data, unsigned len);
void spiRead(spiIF_t *spi, void *data, unsigned len);
void spiWriteRead(spiIF_t *spi, void *dataOut, unsigned lenOut,
                                void *dataIn,  unsigned lenIn   );

int  spi_open_r  (struct _reent *r, const char *path, int flags, int mode );
int  spi_close_r (struct _reent *r, int fd );
long spi_write_r (struct _reent *r, int fd, const void *ptr, int len );
long spi_read_r  (struct _reent *r, int fd, const void *ptr, int len );

/* EXAMPLE *******************************************************************/

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



























