/*******************************************************************************
*
* hardware.h
*
* Rob Laswick
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#if !defined(HARDWARE_H)
#define HARDWARE_H

#include <stdlib.h>
#include "globalDefs.h"

/* ASSERT *********************************************************************/

#if defined(assert)
#undef assert
#endif

extern void assert_(const char *file, const int line);

#define assert(cond) ((cond)? (void) 0 : assert_(__FILE__, __LINE__))

/******************************************************************************/

#define swap32(x) \
    asm volatile ("rev %[out], %[in]" : [out] "=r" (x) : [in] "r" (x))

#define __RAMCODE__ __attribute__ ((long_call, section(".ramcode")))

/* POSIX Interface ************************************************************/

extern int ioctl(int fd, int cmd, int flags);

typedef struct devoptab_s {                        /* Device Operations Table */
    const char *name;
    int  (*open_r )(void *reent, struct devoptab_s *dot, int mode, int flags);
    int  (*ioctl  )(             struct devoptab_s *dot, int cmd,  int flags);
    int  (*close_r)(void *reent, struct devoptab_s *dot);
    long (*write_r)(void *reent, struct devoptab_s *dot, const void *buf,
                                                                       int len);
    long (*read_r )(void *reent, struct devoptab_s *dot, void *buf, int len);
    void *priv;
} devoptab_t;

extern int deviceInstall(
    const char *name,
    int  (*open_r )(void *reent, struct devoptab_s *dot, int mode, int flags),
    int  (*ioctl  )(             struct devoptab_s *dot, int cmd,  int flags),
    int  (*close_r)(void *reent, struct devoptab_s *dot),
    long (*write_r)(void *reent, struct devoptab_s *dot, const void *buf,
                                                                       int len),
    long (*read_r )(void *reent, struct devoptab_s *dot, void *buf, int len),
    void *priv
);

/* INTERRUPTS *****************************************************************/

#define hwInterruptsEnable()  asm volatile ("cpsie i")
#define hwInterruptsDisable() asm volatile ("cpsid i")

extern void hwInstallISRHandler(uint32_t isr, void *isrHandler);

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

                                                        /* IO_IOCTL_ commands */
enum {
    IO_IOCTL_SPI_SET_PORT_PCRS,        /* Selects the SPI in the PORT MUXs */
    IO_IOCTL_SPI_SET_BAUD,             /* Sets the BAUD rate */
    IO_IOCTL_SPI_SET_SCLK_MODE,        /* Sets clock polarity and sample mode.*/
    IO_IOCTL_SPI_SET_FMSZ,             /* */
    IO_IOCTL_SPI_SET_OPTS,             /* */
    IO_IOCTL_SPI_SET_CS,               /* */
    IO_IOCTL_SPI_SET_CS_INACT_STATE,   /* */
    IO_IOCTL_SPI_FLUSH_RX_FIFO,        /* */
    IO_IOCTL_SPI_WRITE_READ,           /* */
    MAX_IO_IOCTRL_SPI_CMDS,            /* Name of this enum is pending... */
};

                                            /* Mode select for the spi module */
typedef enum spiSclkMode_e {
    SPI_SCLK_MODE_0 = 0, /* CPOL = 0, CPHA = 0, ***DEFAULT*** */
    SPI_SCLK_MODE_1,     /* CPOL = 0, CPHA = 1                */
    SPI_SCLK_MODE_2,     /* CPOL = 1, CPHA = 0                */
    SPI_SCLK_MODE_3,     /* CPOL = 1, CPHA = 1                */
    MAX_SPI_SCLK_MODES,
} spiSclkMode_t;

                                       /* Mask for all the spi module options */
typedef enum {
    SPI_OPTS_MASTER         = (1<<0), /* Module is SPI Master */
    SPI_OPTS_LSB_FIRST      = (1<<1), /* LSB of frame is transferred first */
    SPI_OPTS_CONT_SCK_EN    = (1<<2), /* Continuously run the SCK */
    SPI_OPTS_TX_FIFO_DSBL   = (1<<3), /* Disable Tx FIFO */
    SPI_OPTS_RX_FIFO_DSBL   = (1<<4), /* Disable Rx FIFO */
    SPI_OPTS_RX_FIFO_OVR_EN = (1<<5), /* Receive FIFO Overflow Overwrite */
    SPI_OPTS_PCS_CONT       = (1<<6), /* Keep pcs pins asserted between xfers */
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

typedef struct spiWriteRead_s {
    uint8_t *out;
    uint8_t *in;
    unsigned len;
} spiWriteRead_t;

#define SPI_FMSZ_MAX 16
#define SPI_FMSZ_MIN 3

#define DEVOPTAB_SPI0_STR "spi0"
#define DEVOPTAB_SPI1_STR "spi1"
#define DEVOPTAB_SPI2_STR "spi2"

int  spi_install (void);
int  spi_open_r  (void *reent, devoptab_t *dot,  int mode,  int flags);
int  spi_ioctl   (             devoptab_t *dot,  int cmd,   int flags);
int  spi_close_r (void *reent, devoptab_t *dot);
long spi_write_r (void *reent, devoptab_t *dot, const void *buf, int len);
long spi_read_r  (void *reent, devoptab_t *dat,       void *buf, int len);

/* FLASH **********************************************************************/

typedef struct {
} flashConfig_t;

extern int32_t flashInit(const flashConfig_t *cfg);
extern int32_t flashErase(uint32_t addr, uint32_t numBytes);
extern int32_t flashWrite(uint32_t addr, uint32_t *dataPtr, uint32_t numWords);

/* TSI ************************************************************************/
#define TSI_COUNT 16

typedef struct {
    uint32_t pinEnable;
    uint32_t scanc;
    uint16_t prescale;
    uint16_t threshold[TSI_COUNT];
} tsiConfig_t;

#define TSI_SCANC_DEFAULT                (((12-1) << TSI_SCANC_REFCHRG_SHIFT) \
                                        |            TSI_SCANC_CAPTRM_1p0 \
                                        | ((12-1) << TSI_SCANC_EXTCHRG_SHIFT) \
                                        |            TSI_SCANC_DELVOL_600mV \
                                        | (1      << TSI_SCANC_SMOD_SHIFT) \
                                        |            TSI_SCANC_AMCLKS_BUS_CLK \
                                        | (0      << TSI_SCANC_AMPSC_SHIFT))

extern int32_t tsiInit(const tsiConfig_t *cfg);
extern uint32_t tsiRead(const tsiConfig_t *cfg);
extern uint32_t tsiReadRaw(uint32_t pin);

extern int  tsi_install(void);
extern int  tsi_open_r (void *reent, devoptab_t *dot, int mode, int flags);
extern int  tsi_ioctl  (             devoptab_t *dot, int cmd,  int flags);
extern int  tsi_close_r(void *reent, devoptab_t *dot);
extern long tsi_write_r(void *reent, devoptab_t *dot, const void *buf, int len);
extern long tsi_read_r (void *reent, devoptab_t *dot,       void *buf, int len);

/* IO_IOCTL_ commands */
enum {
    IO_IOCTL_TSI_CONFIGURE,         /* Set SCANC register. */
    IO_IOCTL_TSI_SET_PRESCALE,      /* Set TSI prescale. */
    IO_IOCTL_TSI_CONFIGURE_PIN,     /* Configure a TSI pin. */
};

typedef struct tsiConfigure_s {
    uint16_t pin;
    uint16_t threshold;
} tsiConfigure_t;

/* CRC  ***********************************************************************/

                                                        /* IO_IOCTL_ commands */
enum {
    IO_IOCTL_CRC_SET_TOT,         /* Set Type of Transpose */
    IO_IOCTL_CRC_GET_TOT,         /* Get Type of Transpose */
    IO_IOCTL_CRC_SET_TOTR,        /* Set Type of Transpose for Read */
    IO_IOCTL_CRC_GET_TOTR,        /* Get Type of Transpose for Read */
    IO_IOCTL_CRC_SET_FXOR,        /* Set Compliment Read mode (XOR'd CRC) */
    IO_IOCTL_CRC_GET_FXOR,        /* Get Compliment Read mode (XOR'd CRC) */
    IO_IOCTL_CRC_SET_SEED,        /* Set the CRC Seed */
    IO_IOCTL_CRC_GET_SEED,        /* Get the CRC Seed */
    IO_IOCTL_CRC_SET_POLY,        /* Set the CRC Polynomial */
    IO_IOCTL_CRC_GET_POLY,        /* Get the CRC Polynomial */
    IO_IOCTL_CRC_SET_WIDTH,       /* Set the CRC Protocol Width */
    IO_IOCTL_CRC_GET_WIDTH,       /* Get the CRC Protocol Width */
    MAX_IO_IOCTRL_CRC_CMDS
};

                                           /* TOT - CRC Type of Transposition */
typedef enum {
    CRC_TOT_NONE           = 0, /* No transposition */
    CRC_TOT_BITS           = 1, /* Only Bits in bytes are transposed */
    CRC_TOT_BITS_AND_BYTES = 2, /* Bits in bytes and bytes are transposed */
    CRC_TOT_ONLY_BYTES     = 3, /* Only Bytes are transposed */
    MAX_CRC_TOT,
} crcTot_t;

                                  /* TOTR - CRC Type of Transposition for Read*/
typedef enum {
    CRC_TOTR_NONE           = 0, /* No transposition */
    CRC_TOTR_BITS           = 1, /* Only Bits in bytes are transposed */
    CRC_TOTR_BITS_AND_BYTES = 2, /* Bits in bytes and bytes are transposed */
    CRC_TOTR_ONLY_BYTES     = 3, /* Only Bytes are transposed */
    MAX_CRC_TOTR,
} crcTotr_t;

                          /* FXOR - Compliment Read (XOR) of CRC Data Register*/
typedef enum {
    CRC_FXOR_DISABLE = 0, /* Final checksum will not be XOR'd */
    CRC_FXOR_ENABLE  = 1, /* Final checksum XOR'd with all 1's mask */
    MAX_CRC_FXOR,
} crcFxor_t;

                                                 /* WIDTH - CRC Protocol Width*/
typedef enum {
    CRC_WIDTH_16      = 0, /* 16-bit CRC Checksum */
    CRC_WIDTH_32      = 1, /* 32-bit CRC Checksum */
    MAX_CRC_WIDTH,
} crcWidth_t;

#define DEVOPTAB_CRC_STR    "crc"

int  crc_install(void);
int  crc_open_r (void *reent, devoptab_t *dot, int mode, int flags);
int  crc_ioctl  (             devoptab_t *dot, int cmd,  int flags);
int  crc_close_r(void *reent, devoptab_t *dot);
long crc_write_r(void *reent, devoptab_t *dot, const void *buf, int len);
long crc_read_r (void *reent, devoptab_t *dat,       void *buf, int len);

/* MPU ************************************************************************/

enum {
    MPU_ATTR_READ    = BIT_0,
    MPU_ATTR_WRITE   = BIT_1,
    MPU_ATTR_EXECUTE = BIT_2,
};

#define MPU_REGION0_ID 0xDEAFC0DE

typedef struct {
    uint32_t addr;
    uint32_t master;
    uint32_t errorAttr;
    bool32_t writeError;
    bool32_t readError;
} mpuFaultDesc_t;

typedef struct {
    bool32_t enable;
    uint32_t startAddr;             /* Must be  32bit aligned */
    uint32_t endAddr;               /* Must be (32bit aligned - 1) i.e 0x001f */
    uint8_t  attr[MAX_CROSSBAR_MASTERS];
    void (*notifyFn)(const mpuFaultDesc_t *);
} mpuRegion_t;

extern int32_t  mpuEnable(bool32_t enable);
extern int32_t  mpuAddRegion(const mpuRegion_t *regionPtr);
extern int32_t  mpuModifyRegion(int32_t regionId, const mpuRegion_t *regionPtr);
extern bool32_t mpuCheckFaults(void);

/* UART ***********************************************************************/

#define MAX_UARTS     6

#define UART5_PORT    PORTE
#define UART5_PORT_ENABLE SIM_PORTE_ENABLE
#define UART5_RX_PIN  9
#define UART5_TX_PIN  8
#define UART5_RX_MUX  PORT_MUX_ALT3
#define UART5_TX_MUX  PORT_MUX_ALT3

#define UART4_PORT    PORTE
#define UART4_PORT_ENABLE SIM_PORTE_ENABLE
#define UART4_RX_PIN  25
#define UART4_TX_PIN  24
#define UART4_RX_MUX  PORT_MUX_ALT3
#define UART4_TX_MUX  PORT_MUX_ALT3

#define UART3_PORT    PORTC
#define UART3_PORT_ENABLE SIM_PORTC_ENABLE
#define UART3_RX_PIN  16
#define UART3_TX_PIN  17
#define UART3_RX_MUX  PORT_MUX_ALT3
#define UART3_TX_MUX  PORT_MUX_ALT3

#define DEVOPTAB_UART0_STR "uart0"
#define DEVOPTAB_UART1_STR "uart1"
#define DEVOPTAB_UART2_STR "uart2"
#define DEVOPTAB_UART3_STR "uart3"
#define DEVOPTAB_UART4_STR "uart4"
#define DEVOPTAB_UART5_STR "uart5"

int  uart_install(void);
int  uart_open_r (void *reent, devoptab_t *dot, int mode, int flags);
int  uart_ioctl  (             devoptab_t *dot, int cmd,  int flags);
int  uart_close_r(void *reent, devoptab_t *dot);
long uart_write_r(void *reent, devoptab_t *dot, const void *buf, int len);
long uart_read_r (void *reent, devoptab_t *dat,       void *buf, int len);

                                                        /* IO_IOCTL_ commands */
enum {
    IO_IOCTL_UART_CALL_BACK_SET,    /* Register a RX call back function. */
    IO_IOCTL_UART_TERMINATOR_SET,   /* Specify a RX termination character.*/
    IO_IOCTL_UART_FLUSH_RX_FIFO,    /* Flush driver's RX FIFO */
    IO_IOCTL_UART_LOOPBACK_ENABLE,  /* Set the loopback mode */
    IO_IOCTL_UART_BAUD_SET,         /* Set the baud rate */
};

/* WATCH DOG ******************************************************************/
#define WDOG_UNLOCK_KEY_1  0xC520
#define WDOG_UNLOCK_KEY_2  0xD928
#define WDOG_REFRESH_KEY_1 0xA602
#define WDOG_REFRESH_KEY_2 0xB480

extern void watchDogConfig();
extern void watchDogInit();
extern void watchDogKick();
extern void watchDogDisable();

/*******************************************************************************
*
* CCA Hardware Defines
*
*******************************************************************************/
#if defined(FREESCALE_K60N512_TOWER_HW)

/*
 * TODO
 *
 * Hardware FUN FACTS.
 *
 * For each type of  _HW define it would be nice if there was a little
 * summary of all its features and capacities (FLASH size, RAM size,
 * number of UARTS, default, internal, and external clocking optins, etc.).
 */

/* CLOCKS *********************************************************************/

/*
 * On reset, the system clock is defaulted to FEI mode where MCGOUTCLK
 * is derived from the FLL clock, controlled by the 32kHz IRC with a
 * default FLL factor of 640 (=20.48MHz)  See the MCG Modes of Operation
 * table in the device TRM.
 */

/*
 * TODO
 *
 * We should define a hwGetSystemClock(), and use the result everywhere that
 * needs it, rather then relying on a fixed define.  The default fixed
 * define should only be the default power on clock.
 *
 * Jan's clock code would obviously update the hwSystemClock value if it
 * where changed, and/or someone engaged the FLL/PLL, etc.
 */

#define SYSTEM_CLOCK_HZ  20480000
#define    BUS_CLOCK_HZ  20480000

/* LEDS ***********************************************************************/

#define N_LED_ORANGE_PORT PORTA
#define N_LED_ORANGE_PIN  11

#define N_LED_YELLOW_PORT PORTA
#define N_LED_YELLOW_PIN  28

#define N_LED_GREEN_PORT  PORTA
#define N_LED_GREEN_PIN   29

#define N_LED_BLUE_PORT   PORTA
#define N_LED_BLUE_PIN    10

/* SWITCHES *******************************************************************/

#define N_SWITCH_1_PORT  PORTA
#define N_SWITCH_1_PIN   19

/* TSI  ***********************************************************************/

#define TSI_ORANGE_INDEX  5
#define TSI_ORANGE_BIT    BIT_5

#define TSI_YELLOW_INDEX  8
#define TSI_YELLOW_BIT    BIT_8

#define TSI_GREEN_INDEX   7
#define TSI_GREEN_BIT     BIT_7

#define TSI_BLUE_INDEX    9
#define TSI_BLUE_BIT      BIT_9

/******************************************************************************/

#else
#error Undefined Hardware Platform
#endif

#endif
