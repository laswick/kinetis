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

#include <stdio.h>
#include <stdlib.h>

#include "globalDefs.h"
#include "kplatform.h"

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

/* POSIX Interface ***********************************************************/

extern int fdevopen(FILE *stream, const char *file, int flags, int mode);

extern int ioctl(int fd, int cmd, int flags);

enum {                                               /* Major Device Numbers */
    DEV_MAJ_UART,
    DEV_MAJ_SPI,
    DEV_MAJ_TSI,
    DEV_MAJ_CRC,
    DEV_MAJ_ADC,
    DEV_MAJ_SDHC,
    DEV_MAJ_ENET,
};

typedef struct devoptab_s {                       /* Device Operations Table */
    const char *name;
    uint32_t    maj;
    uint32_t    min;
    void       *priv;
} devoptab_t;

extern int deviceRegister (const char *name, uint32_t maj, uint32_t min,
                                                                   void *priv);

typedef struct devlist_s {                             /* Device Driver List */
    int  (*open_r )(void *reent, struct devoptab_s *dot, int mode, int flags);
    int  (*ioctl  )(             struct devoptab_s *dot, int cmd,  int flags);
    int  (*close_r)(void *reent, struct devoptab_s *dot);
    long (*write_r)(void *reent, struct devoptab_s *dot, const void *buf,
                                                                      int len);
    long (*read_r )(void *reent, struct devoptab_s *dot, void *buf, int len);
} devlist_t;

extern int deviceInstall(
    uint32_t maj,
    int  (*open_r )(void *reent, struct devoptab_s *dot, int mode, int flags),
    int  (*ioctl  )(             struct devoptab_s *dot, int cmd,  int flags),
    int  (*close_r)(void *reent, struct devoptab_s *dot),
    long (*write_r)(void *reent, struct devoptab_s *dot, const void *buf,
                                                                      int len),
    long (*read_r )(void *reent, struct devoptab_s *dot, void *buf, int len)
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
    IO_IOCTL_SPI_SET_METHOD,           /* */
    IO_IOCTL_SPI_FLUSH_RX_FIFO,        /* */
    IO_IOCTL_SPI_FLUSH_TX_FIFO,        /* */
    IO_IOCTL_SPI_FLUSH_RX_BUFF,        /* */
    IO_IOCTL_SPI_FLUSH_TX_BUFF,        /* */
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
    SPI_METHOD_POLLED = 0,
    SPI_METHOD_INTERRUPT,
    SPI_METHOD_DMA,
    NUM_SPI_METHODS,
} spiMethods_t;

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
} spiBaudrates_t;

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

#define SPI_FIFO_SIZE   4   /*Both Rx & Tx */

int  spi_install (void);

/* SDHC  ***********************************************************************/

int  sdhc_install (void);

enum {
    IO_IOCTL_SDHC_SET_READ_BLOCK,      /* */
    IO_IOCTL_SDHC_SET_WRITE_BLOCK,     /* */
    IO_IOCTL_SDHC_ERASE_BLOCK,         /* */
    IO_IOCTL_SDHC_GET_CID,             /* */
    IO_IOCTL_SDHC_GET_CSD              /* */
};

typedef union {
    struct {
        uint8_t unused1;
        uint8_t mfgId;
        uint8_t unused2[7];
        uint8_t rev;
        uint8_t sn[4];
        uint8_t mfgDate[2];
    };
    uint32_t raw[4];
} cidReg_t;


typedef struct {
    uint32_t cmdIndx;
    uint32_t cmdType;
    uint32_t rspType;
    uint32_t xferType;
    uint32_t arg;
    uint32_t read;
    uint32_t nBlks;
    uint32_t resp[4];
} sdhcCmd_t;

/*
 * Command Types
 */
enum {
    SDHC_CMD_TYPE_NORMAL,
    SDHC_CMD_TYPE_SUSPEND,
    SDHC_CMD_TYPE_RESUME,
    SDHC_CMD_TYPE_ABORT,
};

/*
 * Response Types
 */
enum {
   SDHC_RSP_TYPE_NONE,
   SDHC_RSP_TYPE_LEN_136,
   SDHC_RSP_TYPE_LEN_48,
   SDHC_RSP_TYPE_LEN_48_BUSY,
};

/* FLASH **********************************************************************/

enum {
    FLASH_BLOCK_0,
    FLASH_BLOCK_1,
#if defined(K60F120) || defined(K70F120)
    FLASH_BLOCK_2,
    FLASH_BLOCK_3,
#endif

    MAX_FLASH_BLOCKS,
};

typedef struct {
} flashConfig_t;

extern int32_t flashInit(const flashConfig_t *cfg);
extern int32_t flashErase(uint32_t addr, uint32_t numBytes);
extern int32_t flashEraseBlock(uint32_t blockNum);
extern int32_t flashWrite(uint32_t addr, uint32_t *dataPtr, uint32_t numWords);
extern int32_t flashSwapInit(void);
extern int32_t flashSwap(void);

/* APPLICATION LOADER *********************************************************/
extern int loader(void);

/* TSI ************************************************************************/

#define TSI_COUNT 16

typedef struct {
    uint32_t pinEnable;
    uint32_t scanc;
    uint16_t prescale;
    uint16_t threshold[TSI_COUNT];
} tsiConfig_t;

#define TSI_SCANC_DEFAULT                (((12-1) << TSI_SCANC_REFCHRG_SHIFT) \
                                        |            TSI_SCANC_CAPTRM_1p2     \
                                        | ((12-1) << TSI_SCANC_EXTCHRG_SHIFT) \
                                        |            TSI_SCANC_DELVOL_600mV   \
                                        | (1      << TSI_SCANC_SMOD_SHIFT)    \
                                        |            TSI_SCANC_AMCLKS_BUS_CLK \
                                        | (0      << TSI_SCANC_AMPSC_SHIFT))

extern int32_t tsiInit(const tsiConfig_t *cfg);
extern uint32_t tsiRead(const tsiConfig_t *cfg);
extern uint32_t tsiReadRaw(uint32_t pin);

extern int  tsi_install(void);

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
    IO_IOCTL_CRC_SET_DWW,         /* Set the CRC Data Write Width */
    IO_IOCTL_CRC_SET_TOT,         /* Set Type of Transpose */
    IO_IOCTL_CRC_SET_TOTR,        /* Set Type of Transpose for Read */
    IO_IOCTL_CRC_SET_FXOR,        /* Set Compliment Read mode (XOR'd CRC) */
    IO_IOCTL_CRC_SET_SEED,        /* Set the CRC Seed */
    IO_IOCTL_CRC_SET_POLY,        /* Set the CRC Polynomial */
    IO_IOCTL_CRC_SET_TYPE,        /* Set the CRC Type, 16/32 bit */
    MAX_IO_IOCTRL_CRC_CMDS
};
                        /* DWW - CRC DATA WRITE WIDTH */
typedef enum {
    CRC_DWW_BYTE   = 0, /* CRC data writes are 1 byte wide               */
                        /*  - CRC16/32 modes, data must be 8bit aligned  */
    CRC_DWW_2BYTE = 1,  /* CRC data writes are 2 bytes wide              */
                        /*  - CRC16/32 modes, data must be 16bit aligned */
    CRC_DWW_4BYTE = 2,  /* CRC data writes are 4 bytes wide              */
                        /*  -CRC32 mode only, data must be 32bit aligned */
    MAX_CRC_DWW,
} crcDww_t;

                                /* TOT - CRC Type of Transposition */
typedef enum {
    CRC_TOT_NONE           = 0, /* No transposition */
    CRC_TOT_BITS_IN_BYTES  = 1, /* Only Bits in bytes are transposed */
    CRC_TOT_BITS_AND_BYTES = 2, /* Bits in bytes and bytes are transposed */
    CRC_TOT_BYTES          = 3, /* Only Bytes are transposed */
    MAX_CRC_TOT,
} crcTot_t;

                                 /* TOTR - CRC Type of Transposition for Read*/
typedef enum {
    CRC_TOTR_NONE           = 0, /* No transposition */
    CRC_TOTR_BITS_IN_BYTES  = 1, /* Only Bits in bytes are transposed */
    CRC_TOTR_BITS_AND_BYTES = 2, /* Bits in bytes and bytes are transposed */
    CRC_TOTR_BYTES     = 3, /* Only Bytes are transposed */
    MAX_CRC_TOTR,
} crcTotr_t;

                          /* FXOR - Compliment Read (XOR) of CRC Data Register*/
typedef enum {
    CRC_FXOR_DISABLE = 0, /* Final checksum will not be XOR'd */
    CRC_FXOR_ENABLE  = 1, /* Final checksum XOR'd with all 1's mask */
    MAX_CRC_FXOR,
} crcFxor_t;

                                        /* Common CRC Polynomial */
typedef enum {
    CRC_POLY_CRC32      = 0x04C11DB7,
    CRC_POLY_CRC16      = 0x8005,
    CRC_POLY_CRC16CITT  = 0x1021,
} crcPoly_t;

                                            /* Common CRC seeds */
typedef enum {
    CRC_SEED_CRC32            = 0xFFFFFFFF,
    CRC_SEED_CRC16            = 0x0000,
    CRC_SEED_CRC16_MODBUS     = 0xFFFF,
    CRC_SEED_CRC16CITT        = 0xFFFF,
    CRC_SEED_CRC16CITT_XMODEM = 0x0000,
} crcSeed_t;

                           /* CRC Protocol Width  */
typedef enum {
    CRC_TYPE_16      = 0, /* 16-bit CRC Checksum */
    CRC_TYPE_32      = 1, /* 32-bit CRC Checksum */
    MAX_CRC_TYPES,
} crcType_t;

#define DEVOPTAB_CRC_STR    "crc"

int  crc_install(void);

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
#define UART5_PORT_ENABLE SIM_SCGC5_PORTE_ENABLE
#define UART5_RX_PIN  9
#define UART5_TX_PIN  8
#define UART5_RX_MUX  PORT_MUX_ALT3
#define UART5_TX_MUX  PORT_MUX_ALT3

#define UART4_PORT    PORTE
#define UART4_PORT_ENABLE SIM_SCGC5_PORTE_ENABLE
#define UART4_RX_PIN  25
#define UART4_TX_PIN  24
#define UART4_RX_MUX  PORT_MUX_ALT3
#define UART4_TX_MUX  PORT_MUX_ALT3

#define UART3_PORT    PORTC
#define UART3_PORT_ENABLE SIM_SCGC5_PORTC_ENABLE
#define UART3_RX_PIN  16
#define UART3_TX_PIN  17
#define UART3_RX_MUX  PORT_MUX_ALT3
#define UART3_TX_MUX  PORT_MUX_ALT3

#define UART2_PORT    PORTE
#define UART2_PORT_ENABLE SIM_SCGC5_PORTE_ENABLE
#define UART2_RX_PIN  17
#define UART2_TX_PIN  16
#define UART2_RX_MUX  PORT_MUX_ALT3
#define UART2_TX_MUX  PORT_MUX_ALT3

#define DEVOPTAB_UART0_STR "uart0"
#define DEVOPTAB_UART1_STR "uart1"
#define DEVOPTAB_UART2_STR "uart2"
#define DEVOPTAB_UART3_STR "uart3"
#define DEVOPTAB_UART4_STR "uart4"
#define DEVOPTAB_UART5_STR "uart5"

int  uart_install(void);

                                                        /* IO_IOCTL_ commands */
enum {
    /* CALL_BACK can be used with ESCAPE & TERMINATOR to manage framed messages.
     * ESCAPE & TERMINATOR have no effect without a registered CALL_BACK */
    IO_IOCTL_UART_CALL_BACK_SET,         /* Register a RX call back function. */
    IO_IOCTL_UART_TERMINATOR_SET,       /* Specify a RX termination character.*/
    IO_IOCTL_UART_ESCAPE_SET,   /* Specify an escape char to start rx capture.*/
    IO_IOCTL_UART_FLUSH_RX_FIFO,                    /* Flush driver's RX FIFO */
    IO_IOCTL_UART_LOOPBACK_ENABLE,                   /* Set the loopback mode */
    IO_IOCTL_UART_BAUD_SET,                              /* Set the baud rate */
    IO_IOCTL_UART_ECHO,                                  /* Enable local echo */
};

/* WATCH DOG ******************************************************************/
extern void watchDogInit(const watchDogConfig_t *wdCfgPtr);
extern void watchDogKick();
extern void watchDogDisable();
/* DAC ************************************************************************/

typedef struct {
    uint8_t low;
    uint8_t high;
} dacData_t;

typedef struct {
    dacData_t data[DAC_DATA_MAX];
    uint8_t sr;
    uint8_t c0;
    uint8_t c1;
    uint8_t c2;
} dac_t;

extern volatile dac_t * const dac0;

extern void dac0Init(void);

/* PIT ************************************************************************/

typedef struct {
    uint32_t loadVal;      /* value to load into pit timer */
    uint32_t currVal;      /* current value of the down counter */
    uint32_t ctrl;
    uint32_t flags;
} pit_t;

enum {
    PIT_0,
    PIT_1,
    PIT_2,
    PIT_3,
    MAX_PIT,
};

typedef struct {
    uint32_t mcr;
    uint32_t reserved[(0x100/4) - 1];
    pit_t pit[4];
} pitCtrl_t;

extern volatile pitCtrl_t * const pitCtrl;

extern void pitInit(int timer, void *isr, uint32_t initCount);

/* FTM ************************************************************************/
/* TODO ftm interface functions coming soon... */
typedef struct {
    uint32_t mode;
    uint16_t initCount;
    uint16_t mod;
    uint8_t  channels[MAX_FTM_CH];
    int      pwmFreq;
    int      dutyScaled[MAX_FTM_CH];
} ftmCfg_t;
enum {
    FTM_MODE_INPUT_CAPTURE,
    FTM_MODE_QUADRATURE_DECODE,
    FTM_MODE_OUTPUT_COMPARE,
    FTM_MODE_PWM,
};

enum {
    FTM_INPUT_CAPTURE_RISING_EDGE,
    FTM_INPUT_CAPTURE_FALLING_EDGE,
    FTM_INPUT_CAPTURE_EITHER_EDGE,
};

enum {
    FTM_OUTPUT_COMPARE_MATCH_SET,
    FTM_OUTPUT_COMPARE_MATCH_CLEAR,
    FTM_OUTPUT_COMPARE_MATCH_TOGGLE,
};

enum {
    FTM_PWM_EDGE_ALIGNED,
    FTM_PWM_CENTER_ALINGNED,
};



extern void ftmInit(int timer, void *isr, ftmCfg_t *ftmCfg);
extern uint16_t ftmRead(int timer);
extern uint16_t ftmWrite(int timer, uint16_t mod, uint16_t initCount);
extern void ftmPwmWrite(int timer, int ch, int32_t duty);
void ftmSetQDPolarity(int timer, int invPolarity);
void ftmSetQDFilter(int timer, uint8_t level);

/* ADC ***********************************************************************/

#define MAX_ADCS     2

/* TODO is there a port enable required for ADC? */
#define ADC0_PORT    PORTA
#define ADC0_PORT_ENABLE SIM_SCGC5_PORTA_ENABLE

#define ADC1_PORT    PORTA
#define ADC1_PORT_ENABLE SIM_SCGC5_PORTA_ENABLE


#if defined(FREESCALE_K60N512_TOWER_HW)
#define ADC_POT_ADC_INPUT ADC_SC1_ADCH_CH20
#endif

#define DEVOPTAB_ADC0_STR "adc0"
#define DEVOPTAB_ADC1_STR "adc1"
int  adc_install(void);
#if 0
int  adc_open_r (void *reent, devoptab_t *dot, int mode, int flags);
int  adc_ioctl  (             devoptab_t *dot, int cmd,  int flags);
int  adc_close_r(void *reent, devoptab_t *dot);
long adc_read_r (void *reent, devoptab_t *dat,       void *buf, int len);
#endif
                                                        /* IO_IOCTL_ commands */
enum {
    IO_IOCTL_ADC_CALIBRATE,
    IO_IOCTL_ADC_OFFSET_SET,
    IO_IOCTL_ADC_PGASET,
    IO_IOCTL_ADC_CLOCK_SELECT,
    IO_IOCTL_ADC_RESOLUTION_SELECT,
    IO_IOCTL_ADC_VREF_SELECT,
    IO_IOCTL_ADC_TRIGGER_SELECT,
    IO_IOCTL_ADC_CHANNEL_SELECT,
    IO_IOCTL_ADC_DIFFERENTIAL_SET,
    IO_IOCTL_ADC_CONVERSION_CONTINUOUS,
    IO_IOCTL_ADC_CONVERSION_TIME_SELECT,
    IO_IOCTL_ADC_AVERAGE_SELECT,

    IO_IOCTL_ADC_COMPARE_ENABLE,
    IO_IOCTL_ADC_COMPARE_HIGH_LOW_SET,
    IO_IOCTL_ADC_COMPARE_RANGE_SET,
    IO_IOCTL_ADC_COMPARE_VALUES_SET,


    IO_IOCTL_ADC_CALL_BACK_SET,     /* Register a call back function. */
    IO_IOCTL_ADC_SAMPLE_SIZE_SET,   /* Specify number of samples.*/
    IO_IOCTL_ADC_FLUSH_FIFO,       /* Flush driver's FIFO */
};

enum {
    IO_IOCTL_ADC_RESOLUTION_FLAGS_8BIT,
    IO_IOCTL_ADC_RESOLUTION_FLAGS_12BIT,
    IO_IOCTL_ADC_RESOLUTION_FLAGS_10BIT,
    IO_IOCTL_ADC_RESOLUTION_FLAGS_16BIT,
};

#define IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_B (1 << 31)
#define IO_IOCTL_ADC_CHANNEL_FLAGS_REGISTER_A (0 << 31)
#define IO_IOCTL_ADC_CHANNEL_FLAGS_CH_MASK  0x1F
enum {
    IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_SHORT_SAMPLE   = -1,

    /* These map 1:1 with the ADLSTS reg value.  If you change them:
    * 1. Shame on you.
    * 2. You will need to set up a switch LUT in adc.c
   */
    IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_20 =  0,
    IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_12,
    IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_6,
    IO_IOCTL_ADC_CONVERSION_TIME_FLAGS_ADLSTS_ADCK_2,
};

enum {
    /* These map 1:1 with the reg value.  If you change them:
    * 1. Shame on you.
    * 2. You will need to set up a switch LUT in adc.c
    */

    IO_IOCTL_ADC_FLAGS_AVGS_4,
    IO_IOCTL_ADC_FLAGS_AVGS_8,
    IO_IOCTL_ADC_FLAGS_AVGS_16,
    IO_IOCTL_ADC_FLAGS_AVGS_32,
};

enum {
    /* These map 1:1 with the reg value. */
    IO_IOCTL_ADC_RES_FLAGS_8_BIT,  /* 2's complitment 9-bit output in DIFF */
    IO_IOCTL_ADC_RES_FLAGS_12_BIT, /* 2's complitment 13-bit output in DIFF */
    IO_IOCTL_ADC_RES_FLAGS_10_BIT, /* 2's complitment 11-bit output in DIFF */
    IO_IOCTL_ADC_RES_FLAGS_16_BIT, /* 2's complitment 16-bit output in DIFF */
};

enum {
    /* These map 1:1 with the reg value. */
    IO_IOCTL_ADC_FLAGS_ADICLK_BUS,
    IO_IOCTL_ADC_FLAGS_ADICLK_BUS_DIV_2,
    IO_IOCTL_ADC_FLAGS_ADICLK_ALTCLK,
    IO_IOCTL_ADC_FLAGS_ADICLK_ADACK,
};

enum {
     IO_IOCTL_ADC_TRIGGER_SELECT_SW,
     IO_IOCTL_ADC_TRIGGER_SELECT_HW,
};

enum {
     IO_IOCTL_ADC_DIFF_FLAGS_SINGLE_ENDED = FALSE,
     IO_IOCTL_ADC_DIFF_FLAGS_DIFFERENTIAL = TRUE,
};
#define IO_IOCTL_ADC_DIFF_FLAGS_MASK  0xF
enum {
    IO_IOCTL_ADC_VREF_FLAGS_DEFAULT,
    IO_IOCTL_ADC_VREF_FLAGS_ALT,
};

enum {
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_1,
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_2,
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_4,
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_8,
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_16,
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_32,
    IO_IOCTL_ADC_PGA_FLAGS_GAIN_64,
};

enum {
    IO_IOCTL_ADC_COMPARE_HIGH_LOW_FLAG_LESS,
    IO_IOCTL_ADC_COMPARE_HIGH_LOW_FLAG_GREATER,
};
#define IO_IOCTL_ADC_COMPARE_VAL_MASK 0xFFFF
#define IO_IOCTL_ADC_COMPARE_VAL2_SHIFT 16

/* ENET ***********************************************************************/

#define DEVOPTAB_ENET0_STR "eth0"

int  enet_install (void);
int eth_hwaddr_match (uint8_t* a1, uint8_t* a2);
int eth_hwaddr_zero (uint8_t* a1);
int eth_hwaddr_eff (uint8_t* a1);
void eth_hwaddr_copy (uint8_t* dest, uint8_t* src);

/* Need these swap macros to deal with little/big endian issues */
#define BSWAP32(inval) (__builtin_bswap32((uint32_t)(inval)))
#define BSWAP16(inval) ((uint16_t)__builtin_bswap32(((uint32_t)(inval)) << 16 ))

enum {
    IO_IOCTL_ENET_SET_MAC_ADDRESS,      /* You have to set a MAC address */
    IO_IOCTL_ENET_SET_ENET_STATE,       /* Turn ethernet on/off - Must have a MAC */
    IO_IOCTL_ENET_GET_ENET_STATE,       /* Return ethernet on / off state */
    IO_IOCTL_ENET_GET_DETAILED_ERROR,   /* Return more detailed error info */
    IO_IOCTL_ENET_SET_AUTONEGOTIATE,    /* Set Phy autonegotiate on/off */
    IO_IOCTL_ENET_SET_SPEED,            /* Set desired link speed */
    IO_IOCTL_ENET_SET_DUPLEX,           /* Set desired duplex */
    IO_IOCTL_ENET_GET_PHY_CONFIG,       /* Return current phy config struct */
    IO_IOCTL_ENET_GET_PHY_STATUS,       /* Return current phy status struct */
    IO_IOCTL_ENET_GET_PHY_REG,          /* Read a raw phy register */
    IO_IOCTL_ENET_GET_LAST_RXBD,        /* Return contents of last rxbd */
    MAX_IO_IOCTRL_ENET_CMDS,            
};

typedef enum enet_state_e {
    ENET_OFF,
    ENET_ON
} enet_state_t;

typedef enum enet_autoneg_e {
    ENET_AUTONEG_ON,
    ENET_AUTONEG_OFF
} enet_autoneg_t;

typedef enum enet_link_e {
    ENET_LINK_DOWN,
    ENET_LINK_UP
} enet_link_t;

typedef enum enet_speed_e {
    ENET_10BASET,
    ENET_100BASET
} enet_speed_t;

typedef enum enet_duplex_e {
    ENET_DUPLEX_HALF,
    ENET_DUPLEX_FULL
} enet_duplex_t;

typedef enum enet_loopback_e {
    ENET_LOOPBACK_INT,
    ENET_LOOPBACK_EXT,
    ENET_LOOPBACK_OFF
} enet_loopback_t;

typedef enum enet_prom_e {
    ENET_PROM_OFF,
    ENET_PROM_ON
} enet_prom_t;

typedef struct enet_cfg_s {
    uint8_t         phy_addr;
    enet_autoneg_t  autoneg;
    enet_speed_t    speed;
    enet_duplex_t   duplex;
    enet_loopback_t loopback;
    enet_prom_t     prom;
    uint8_t         mac_addr[6];
} enet_cfg_t;

typedef struct enet_status_s {
    enet_state_t    on_off;
    enet_link_t     link;
    enet_speed_t    speed;
    enet_duplex_t   duplex;
} enet_status_t;

typedef struct enet_descr_s {
    uint16_t status;    /* control and status */
    uint16_t length;    /* transfer length */
    uint8_t  *buf_addr; /* buffer address NOTE Big Endian!!!*/
} enet_descr_t;

/*******************************************************************************
*
* CCA Hardware Defines
*
*******************************************************************************/
#if defined(FREESCALE_K60N512_TOWER_HW) || \
    defined(FREESCALE_K60F120_TOWER_HW) || \
    defined(FREESCALE_K70F120_TOWER_HW)
  
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

#define BUS_CLOCK_HZ            20480000

#define SYSTEM_CLOCK_HZ_DFLT    BUS_CLOCK_HZ        /* Default power-on clock */
#define SYSTEM_DIVIDER_DFLT     DIVIDE_BY_1
#define BUS_DIVIDER_DFLT        DIVIDE_BY_1
#define FLEXBUS_DIVIDER_DFLT    DIVIDE_BY_2
#define FLASH_DIVIDER_DFLT      DIVIDE_BY_2

#if defined(K60N512)

#define MAX_SYSTEM_FREQ         100000000
#define MAX_BUS_FREQ            50000000
#define MAX_FLEXBUS_FREQ        MAX_BUS_FREQ
#define MAX_FLASH_FREQ          25000000
    
#elif defined(K60F120) || defined(K70F120)

#define MAX_SYSTEM_FREQ         120000000
#define MAX_BUS_FREQ            60000000
#define MAX_FLEXBUS_FREQ        50000000
#define MAX_FLASH_FREQ          25000000
    
#endif

        /* Dividers are used to configure the system/bus/flexbus/flash clocks */
typedef enum {
    DIVIDE_BY_1 = 0,
    DIVIDE_BY_2,
    DIVIDE_BY_3,
    DIVIDE_BY_4,
    DIVIDE_BY_5,
    DIVIDE_BY_6,
    DIVIDE_BY_7,
    DIVIDE_BY_8,
    DIVIDE_BY_9,
    DIVIDE_BY_10,
    DIVIDE_BY_11,
    DIVIDE_BY_12,
    DIVIDE_BY_13,
    DIVIDE_BY_14,
    DIVIDE_BY_15,
    DIVIDE_BY_16,
    MAX_DIVIDER,
} divider_t;


typedef enum {
    MCG_PLL_EXTERNAL_120MHZ,
    MCG_PLL_EXTERNAL_100MHZ,
    MCG_PLL_EXTERNAL_48MHZ,
    MCG_FLL_INTERNAL_24MHZ,
    MAX_MCG_CLOCK_OPTIONS,
} clockConfig_t;

extern void clockSetDividers(divider_t systemDiv, divider_t busDiv,
                                      divider_t flexBusDiv, divider_t flashDiv);
extern uint32_t clockGetFreq(clockSource_t cs);

extern void clockConfigMcgOut(clockConfig_t cc);
extern void clockConfigMcgIr();
extern void clockConfigMcgFf();
extern void clockConfigMcgFll();
extern void clockConfigMcgPll();

extern void clockConfigOsc();
extern void clockConfigOsc32k();
extern void clockConfigOscEr();

extern void clockConfigEr32k();

extern void clockConfigRtc();

extern void clockConfigLpo();

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

#define N_SWITCH_0_PORT  PORTE
#define N_SWITCH_0_PIN   26

#if defined(FREESCALE_K60N512_TOWER_HW) || defined(FREESCALE_K60F120_TOWER_HW)
#define N_SWITCH_1_PORT  PORTA
#define N_SWITCH_1_PIN   19
#elif defined(FREESCALE_K70F120_TOWER_HW)
#define N_SWITCH_1_PORT  PORTD
#define N_SWITCH_1_PIN   0
#endif

/* UART for STDIO  ************************************************************/
#if defined(FREESCALE_K60N512_TOWER_HW)
#define STDIO_UART "uart3"
#elif defined(FREESCALE_K60F120_TOWER_HW)
#define STDIO_UART "uart5"
#elif defined(FREESCALE_K70F120_TOWER_HW)
#define STDIO_UART "uart2"
#endif

/* TSI  ***********************************************************************/

#define TSI_ORANGE_INDEX  5
#define TSI_ORANGE_BIT    BIT_5

#define TSI_YELLOW_INDEX  8
#define TSI_YELLOW_BIT    BIT_8

#define TSI_GREEN_INDEX   7
#define TSI_GREEN_BIT     BIT_7

#define TSI_BLUE_INDEX    9
#define TSI_BLUE_BIT      BIT_9


/* ADC ***********************************************************************/
#define ADC_ALTCLK_SOURCE CLOCKS_OSCERCLK

enum {
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DP0        = 0,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DP1        = 1,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_PGA0_DP1        = 2,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DP3        = 3,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED1       = 4,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED2       = 5,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED3       = 6,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED4       = 7,

    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE4B       = 4,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE5B       = 5,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE6B       = 6,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE7B       = 7,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE8        = 8,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE9        = 9,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE10       = 10,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE11       = 11,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE12       = 12,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE13       = 13,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE14       = 14,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE15       = 15,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE16       = 16,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE17       = 17,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_SE18       = 18,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DM0        = 19,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_ADC0_DM1        = 20,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED5       = 21,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED6       = 22,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_12_BIT_DAC0     = 23,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED7       = 24,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED8       = 25,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_TEMP_SENSOR     = 26,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_BANDGAP         = 27,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_RESERVED9       = 28,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_VREFH           = 29,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_VREFL           = 30,
    IO_IOCTL_ADC0_CHANNEL_FLAGS_MODULE_DISABLED = 31,
};

enum {
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DP0        = 0,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DP1        = 1,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_PGA1_DP1        = 2,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DP3        = 3,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE4A       = 4,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE5A       = 5,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE6A       = 6,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE7A       = 7,

    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE4B       = 4,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE5B       = 5,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE6B       = 6,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE7B       = 7,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE8        = 8,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE9        = 9,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE10       = 10,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE11       = 11,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE12       = 12,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE13       = 13,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE14       = 14,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE15       = 15,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE16       = 16,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_SE17       = 17,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_VREF       = 18,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DM0        = 19,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_ADC1_DM1        = 20,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_RESERVED1       = 21,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_RESERVED2       = 22,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_12_BIT_DAC0     = 23,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_RESERVED3       = 24,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_RESERVED4       = 25,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_TEMP_SENSOR     = 26,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_BANDGAP         = 27,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_RESERVED5       = 28,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_VREFH           = 29,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_VREFL           = 30,
    IO_IOCTL_ADC1_CHANNEL_FLAGS_MODULE_DISABLED = 31,
};

/******************************************************************************/
/*
 * Phy definitions in here because they are not part of the kinetis
 * they are in the PHY on the Tower Serial Board (Micrel)
 */

#define MII_TIMEOUT         0x1FFFF     /* Timeout when talking MII to PHY */
#define MII_LINK_TIMEOUT    0x1FFFF     /* Timeout when resetting PHY */

/* MII Register Addresses */
#define PHY_BMCR                    (0x00)
#define PHY_BMSR                    (0x01)
#define PHY_PHYIDR1                 (0x02)
#define PHY_PHYIDR2                 (0x03)
#define PHY_ANAR                    (0x04)
#define PHY_ANLPAR                  (0x05)
#define PHY_ANLPARNP                (0x05)
#define PHY_ANER                    (0x06)
#define PHY_ANNPTR                  (0x07)
#define PHY_PHYSTS                  (0x10)
#define PHY_MICR                    (0x11)
#define PHY_MISR                    (0x12)
#define PHY_PAGESEL                 (0x13)

/*TWR definition: Micrel*/
#define PHY_PHYCTRL1                (0x1E)
#define PHY_PHYCTRL2                (0x1F)

/* Bit definitions and macros for PHY_BMCR */
#define PHY_BMCR_RESET              (0x8000)
#define PHY_BMCR_LOOP               (0x4000)
#define PHY_BMCR_SPEED              (0x2000)
#define PHY_BMCR_AN_ENABLE          (0x1000)
#define PHY_BMCR_POWERDOWN          (0x0800)
#define PHY_BMCR_ISOLATE            (0x0400)
#define PHY_BMCR_AN_RESTART         (0x0200)
#define PHY_BMCR_FDX                (0x0100)
#define PHY_BMCR_COL_TEST           (0x0080)

/* Bit definitions and macros for PHY_BMSR */
#define PHY_BMSR_100BT4             (0x8000)
#define PHY_BMSR_100BTX_FDX         (0x4000)
#define PHY_BMSR_100BTX             (0x2000)
#define PHY_BMSR_10BT_FDX           (0x1000)
#define PHY_BMSR_10BT               (0x0800)
#define PHY_BMSR_NO_PREAMBLE        (0x0040)
#define PHY_BMSR_AN_COMPLETE        (0x0020)
#define PHY_BMSR_REMOTE_FAULT       (0x0010)
#define PHY_BMSR_AN_ABILITY         (0x0008)
#define PHY_BMSR_LINK               (0x0004)
#define PHY_BMSR_JABBER             (0x0002)
#define PHY_BMSR_EXTENDED           (0x0001)

/* Bit definitions and macros for PHY_ANAR */
#define PHY_ANAR_NEXT_PAGE          (0x8001)
#define PHY_ANAR_REM_FAULT          (0x2001)
#define PHY_ANAR_PAUSE              (0x0401)
#define PHY_ANAR_100BT4             (0x0201)
#define PHY_ANAR_100BTX_FDX         (0x0101)
#define PHY_ANAR_100BTX             (0x0081)
#define PHY_ANAR_10BT_FDX           (0x0041)
#define PHY_ANAR_10BT               (0x0021)
#define PHY_ANAR_802_3              (0x0001)

/* Bit definitions and macros for PHY_ANLPAR */
#define PHY_ANLPAR_NEXT_PAGE        (0x8000)
#define PHY_ANLPAR_ACK              (0x4000)
#define PHY_ANLPAR_REM_FAULT        (0x2000)
#define PHY_ANLPAR_PAUSE            (0x0400)
#define PHY_ANLPAR_100BT4           (0x0200)
#define PHY_ANLPAR_100BTX_FDX       (0x0100)
#define PHY_ANLPAR_100BTX           (0x0080)
#define PHY_ANLPAR_10BTX_FDX        (0x0040)
#define PHY_ANLPAR_10BT             (0x0020)


/* Bit definition and macros for PHY_PHYCTRL1 */
#define PHY_PHYCTRL1_LED_MASK       (0xC000)
#define PHY_PHYCTRL1_POLARITY       (0x2000)
#define PHY_PHYCTRL1_MDX_STATE      (0x0800)
#define PHY_PHYCTRL1_REMOTE_LOOP    (0x0080)

/* Bit definition and macros for PHY_PHYCTRL2 */
#define PHY_PHYCTRL2_HP_MDIX        (0x8000)
#define PHY_PHYCTRL2_MDIX_SELECT    (0x4000)
#define PHY_PHYCTRL2_PAIRSWAP_DIS   (0x2000)
#define PHY_PHYCTRL2_ENERGY_DET     (0x1000)
#define PHY_PHYCTRL2_FORCE_LINK     (0x0800)
#define PHY_PHYCTRL2_POWER_SAVING   (0x0400)
#define PHY_PHYCTRL2_INT_LEVEL      (0x0200)
#define PHY_PHYCTRL2_EN_JABBER      (0x0100)
#define PHY_PHYCTRL2_AUTONEG_CMPLT  (0x0080)
#define PHY_PHYCTRL2_ENABLE_PAUSE   (0x0040)
#define PHY_PHYCTRL2_PHY_ISOLATE    (0x0020)
#define PHY_PHYCTRL2_OP_MOD_MASK    (0x001C)
#define PHY_PHYCTRL2_EN_SQE_TEST    (0x0002)
#define PHY_PHYCTRL2_DATA_SCRAM_DIS (0x0001)


/* Bit definitions of PHY_PHYCTRL2_OP_MOD_MASK */
#define PHY_PHYCTRL2_OP_MOD_SHIFT             2
#define PHY_PHYCTRL2_MODE_OP_MOD_STILL_NEG    0
#define PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_HD    1
#define PHY_PHYCTRL2_MODE_OP_MOD_100MBPS_HD   2
#define PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_FD    5
#define PHY_PHYCTRL2_MODE_OP_MOD_100MBPS_FD   6


/******************************************************************************/
#endif /* defined(FREESCALE_K60N512_TOWER_HW) || 
          defined(FREESCALE_K60F120_TOWER_HW) || 
          defined(FREESCALE_K70F120_TOWER_HW) */

#endif /* !defined(HARDWARE_H) */

