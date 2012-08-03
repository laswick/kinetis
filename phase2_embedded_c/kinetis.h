/*******************************************************************************
*
* kinetis.h
*
*******************************************************************************/
#if !defined(KINETIS_H)
#define KINETIS_H

#include "globalDefs.h"

/*******************************************************************************
* SIM
*******************************************************************************/
#define SIM_SDID (*(volatile uint32_t *) 0x40048024)

#define SIM_REVID_SHIFT 12
#define SIM_REVID_MASK (0x4 << SIM_REVID_SHIFT)     /* Device Revision Number */

#define SIM_FAMID_SHIFT 4
#define SIM_FAMID_MASK (0x7 << SIM_FAMID_SHIFT)          /* Kinetis Family Id */
#define SIM_FAMID_STRINGS "K10", "K20", "K30", "K40", \
                          "K60", "K70", "K50 and K52", "K51 and K53"

#define SIM_PINID_SHIFT 0
#define SIM_PINID_MASK (0xf << SIM_PINID_SHIFT)                 /* Pincout Id */

/* System Clock Gate Control Registers */
#define SIM_SCGC1 (*(volatile uint32_t *) 0x40048028)
#define SIM_SCGC4 (*(volatile uint32_t *) 0x40048034)
#define SIM_UART5_ENABLE  BIT_11
#define SIM_UART4_ENABLE  BIT_10
#define SIM_EWM_ENABLE    BIT_1
#define SIM_CMT_ENABLE    BIT_2
#define SIM_I2C0_ENABLE   BIT_6
#define SIM_I2C1_ENABLE   BIT_7
#define SIM_UART0_ENABLE  BIT_10
#define SIM_UART1_ENABLE  BIT_11
#define SIM_UART2_ENABLE  BIT_12
#define SIM_UART3_ENABLE  BIT_13
#define SIM_USBOTG_ENABLE BIT_18
#define SIM_CMP_ENABLE    BIT_19
#define SIM_VREF_ENABLE   BIT_20
#define SIM_LLWU_ENABLE   BIT_28
#define SIM_SCGC6_ADDR  0x4004803C
#define SIM_SCGC6_PTR (volatile uint32_t *) SIM_SCGC6_ADDR
#define SIM_SCGC6 (*(volatile uint32_t *) SIM_SCGC6_ADDR)
#define SIM_SCGC6_SPI0_ENABLE  BIT_12
#define SIM_SCGC6_SPI1_ENABLE  BIT_13
#define SIM_SCGC3_ADDR  0x40048030
#define SIM_SCGC3_PTR (volatile uint32_t *) SIM_SCGC3_ADDR
#define SIM_SCGC3 (*(volatile uint32_t *) SIM_SCGC3_ADDR)
#define SIM_SCGC3_SPI2_ENABLE  BIT_12
#define SIM_SCGC5_ADDR  0x40048038
#define SIM_SCGC5_PTR (volatile uint32_t) * SIM_SCGC5_ADDR
#define SIM_SCGC5 (*(volatile uint32_t *) SIM_SCGC5_ADDR)
#define SIM_PORTA_ENABLE BIT_9
#define SIM_PORTB_ENABLE BIT_10
#define SIM_PORTC_ENABLE BIT_11
#define SIM_PORTD_ENABLE BIT_12
#define SIM_PORTE_ENABLE BIT_13

/*******************************************************************************
* PORT CONTROLLER
*******************************************************************************/
#define PORT_PCR(port, pin) (*(volatile uint32_t *) (port + (4 * pin)))

#define PORT_ISF BIT_24                              /* Interrupt Status Mask */

enum {                                             /* Interrupt Configuration */
    PORT_IRQC_DISABLED         =  0 << 16,
    PORT_IRQC_DMA_RISING_EDGE  =  1 << 16,
    PORT_IRQC_DMA_FALLING_EDGE =  2 << 16,
    PORT_IRQC_DMA_EITHER_EDGE  =  3 << 16,
    PORT_IRQC_INT_WHEN_LOW     =  8 << 16,
    PORT_IRQC_INT_RISING_EDGE  =  9 << 16,
    PORT_IRQC_INT_FALLING_EDGE = 10 << 16,
    PORT_IRQC_INT_EITHER_EDGE  = 11 << 16,
    PORT_IRQC_INT_WHEN_HIGH    = 12 << 16,
};

#define PORT_LK BIT_15                                       /* Lock Register */

enum {                                                     /* Pin Mux Control */
    PORT_MUX_DISABLED = 0 << 8,
    PORT_MUX_ANALOG   = 0 << 8,
    PORT_MUX_GPIO     = 1 << 8,
    PORT_MUX_ALT1     = 1 << 8,              /* ALT1-7 Chip Specific Function */
    PORT_MUX_ALT2     = 2 << 8,
    PORT_MUX_ALT3     = 3 << 8,
    PORT_MUX_ALT4     = 4 << 8,
    PORT_MUX_ALT5     = 5 << 8,
    PORT_MUX_ALT6     = 6 << 8,
    PORT_MUX_ALT7     = 7 << 8,
};

#define PORT_DSE BIT_6                               /* Drive Strength Enable */
#define PORT_ODE BIT_5                                   /* Open Drain Enable */
#define PORT_PFE BIT_4                               /* Passive Filter Enable */
#define PORT_SRE BIT_2                    /* Slew Rate Enable (0:fast,1:slow) */
#define PORT_PE  BIT_1                                         /* Pull Enable */
#define PORT_PS  BIT_0                           /* Pull Select (0:down,1:up) */

#define PORT_PULLUP_ENABLE   (PORT_PE | PORT_PS)
#define PORT_PULLDOWN_ENABLE (PORT_PE)

#define PORTA_BASE_ADDR 0x40049000
#define PORTB_BASE_ADDR 0x4004a000
#define PORTC_BASE_ADDR 0x4004b000
#define PORTD_BASE_ADDR 0x4004c000
#define PORTE_BASE_ADDR 0x4004d000

#define PORTA PORTA_BASE_ADDR
#define PORTB PORTB_BASE_ADDR
#define PORTC PORTC_BASE_ADDR
#define PORTD PORTD_BASE_ADDR
#define PORTE PORTE_BASE_ADDR

/*******************************************************************************
* GPIO
*
* PDOR = Port Data Output Register
* PSOR = Port Set Output Register
* PCOR = Port Clear Output Register
* PTOR = Port Toggle Register
* PDIR = Port Data Input Register
* PDDR = Port Data Direction Register
*
*******************************************************************************/
typedef struct {
    uint32_t pdor;
    uint32_t psor;
    uint32_t pcor;
    uint32_t ptor;
    uint32_t pdir;
    uint32_t pddr;
} gpioPort_t;

#define GPIOA_BASE_ADDR 0x400ff000
#define GPIOB_BASE_ADDR 0x400ff040
#define GPIOC_BASE_ADDR 0x400ff080
#define GPIOD_BASE_ADDR 0x400ff0c0
#define GPIOE_BASE_ADDR 0x400ff100

#define gpioPortA ((volatile gpioPort_t *) GPIOA_BASE_ADDR)
#define gpioPortB ((volatile gpioPort_t *) GPIOB_BASE_ADDR)
#define gpioPortC ((volatile gpioPort_t *) GPIOC_BASE_ADDR)
#define gpioPortD ((volatile gpioPort_t *) GPIOD_BASE_ADDR)
#define gpioPortE ((volatile gpioPort_t *) GPIOE_BASE_ADDR)

#define GPIOA_PDOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x00))
#define GPIOA_PSOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x04))
#define GPIOA_PCOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x08))
#define GPIOA_PTOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x0c))
#define GPIOA_PDIR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x10))
#define GPIOA_PDDR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x14))

#define GPIOB_PDOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x00))
#define GPIOB_PSOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x04))
#define GPIOB_PCOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x08))
#define GPIOB_PTOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x0c))
#define GPIOB_PDIR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x10))
#define GPIOB_PDDR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x14))

#define GPIOC_PDOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x00))
#define GPIOC_PSOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x04))
#define GPIOC_PCOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x08))
#define GPIOC_PTOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x0c))
#define GPIOC_PDIR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x10))
#define GPIOC_PDDR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x14))

#define GPIOD_PDOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x00))
#define GPIOD_PSOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x04))
#define GPIOD_PCOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x08))
#define GPIOD_PTOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x0c))
#define GPIOD_PDIR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x10))
#define GPIOD_PDDR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x14))

#define GPIOE_PDOR (*(volatile uint32_t *) (GPIOE_BASE_ADDR + 0x00))
#define GPIOE_PSOR (*(volatile uint32_t *) (GPIOE_BASE_ADDR + 0x04))
#define GPIOE_PCOR (*(volatile uint32_t *) (GPIOE_BASE_ADDR + 0x08))
#define GPIOE_PTOR (*(volatile uint32_t *) (GPIOE_BASE_ADDR + 0x0c))
#define GPIOE_PDIR (*(volatile uint32_t *) (GPIOE_BASE_ADDR + 0x10))
#define GPIOE_PDDR (*(volatile uint32_t *) (GPIOE_BASE_ADDR + 0x14))

/*******************************************************************************
* UART
*
* BDH   = Baud Rate Register High
* BDL   = Baus Rate Register Low
* C1    = Control Register 1
* C2    = Control Register 2
* S1    = Status Register 1
* S2    = Status Register 2
* C3    = Control Register 3
* D     = Data Register
* MA1   = Match Address Registers 1
* MA2   = Match Address Registers 2
* C4    = Control Register 4
* C5    = Control Register 5
* ED    = Extended Data Register
* MODEM = Modem Register
* IR    = Infrared Register
* PFIFO = FIFO Parameters
* CFIFO = FIFO Control Register
* SFIFO = FIFO Status Register
* TWFIFO = FIFO Transmit Watermark
* TCFIFO = FIFO Transmit Count
* RWFIFO = Receive Watermark
* RCFIFO = Receive Count
* C7816  = 7816 Control Register
* IE7816 = 7816 Interrupt Enable Register
* IS7816 = 7816 Interrupt Status Register
* WP7816T0 = 7816 Wait Parameter Register
* WP7816T1 = 7816 Wait Parameter Register
* WN7816   = 7816 Wait N Register
* WF7816   = 7816 Wait FD Register
* ET7816   = 7816 Error Threshold Register
* TL7816   = 7816 Transmit Length Register
*
*******************************************************************************/
typedef struct {
    uint8_t bdh;      /* Baud Rate Register High,            0x0 */
    uint8_t bdl;      /* Baus Rate Register Low,             0x1 */
    uint8_t c1;       /* Control Register 1,                 0x2 */
    uint8_t c2;       /* Control Register 2,                 0x3 */
    uint8_t s1;       /* Status Register 1,                  0x4 */
    uint8_t s2;       /* Status Register 2,                  0x5 */
    uint8_t c3;       /* Control Register 3,                 0x6 */
    uint8_t d;        /* Data Register,                      0x7 */
    uint8_t ma1;      /* Match Address Registers 1,          0x8 */
    uint8_t ma2;      /* Match Address Registers 2,          0x9 */
    uint8_t c4;       /* Control Register 4,                 0xa */
    uint8_t c5;       /* Control Register 5,                 0xb */
    uint8_t ed;       /* Extended Data Register,             0xc */
    uint8_t modem;    /* Modem Register,                     0xd */
    uint8_t ir;       /* Infrared Register,                  0xe */
    uint8_t spare0;
    uint8_t pfifo;    /* FIFO Parameters,                    0x10 */
    uint8_t cfifo;    /* FIFO Control Register,              0x11 */
    uint8_t sfifo;    /* FIFO Status Register,               0x12 */
    uint8_t twfifo;   /* FIFO Transmit Watermark,            0x13 */
    uint8_t tcfifo;   /* FIFO Transmit Count,                0x14 */
    uint8_t rwfifo;   /* Receive Watermark,                  0x15 */
    uint8_t rcfifo;   /* Receive Count,                      0x16 */
    uint8_t spare1;
    uint8_t c7816;    /* 7816 Control Register,              0x18 */
    uint8_t ie7816;   /* 7816 Interrupt Enable Register,     0x19 */
    uint8_t is7816;   /* 7816 Interrupt Status Register,     0x1a */
    uint8_t wp7816tx; /* 7816 Wait Parameter Register T0/T1, 0x1b */
    uint8_t wn7816;   /* 7816 Wait N Register,               0x1c */
    uint8_t wf7816;   /* 7816 Wait FD Register,              0x1d */
    uint8_t et7816;   /* 7816 Error Threshold Register,      0x1e */
    uint8_t tl7816;   /* 7816 Transmit Length Register,      0x1f */
} uartPort_t;

#define UART0_BASE_ADDR 0x4006a000
#define UART1_BASE_ADDR 0x4006b000
#define UART2_BASE_ADDR 0x4006c000
#define UART3_BASE_ADDR 0x4006d000
#define UART4_BASE_ADDR 0x400ea000
#define UART5_BASE_ADDR 0x400eb000 /* MK60DN512ZVMD10 */

#define UART0 UART0_BASE_ADDR
#define UART1 UART1_BASE_ADDR
#define UART2 UART2_BASE_ADDR
#define UART3 UART3_BASE_ADDR
#define UART4 UART4_BASE_ADDR
#define UART5 UART5_BASE_ADDR

/* BDH */
#define UART_BDH_RX_ACTIVE_INT_ENABLE BIT_6
#define UART_BDH_LIN_BREAK_INT_ENABLE BIT_7
#define UART_BDH_SBR_MASK 0x1f00
#define UART_BDH_SBR_SHIFT 8

/* BDL */
#define UART_BDL_SBR_MASK 0xff


/* C1 */
#define UART_C1_PARITY_ODD            BIT_0
#define UART_C1_PARITY_ENABLE         BIT_1
#define UART_C1_IDLE_LINE_AFTER_STOP  BIT_2
#define UART_C1_ADDRESS_MARK_WAKE     BIT_3
#define UART_C1_9_BIT_MODE            BIT_4
#define UART_C1_SINGLE_WIRE_LOOP_BACK BIT_5
#define UART_C1_UART_FREEZE_IN_WAIT   BIT_6
#define UART_C1_LOOP_BACK             BIT_7



/* C2 */
#define UART_C2_SEND_BREAK              BIT_0
#define UART_C2_RX_WAKEUP               BIT_1
#define UART_C2_RX_ENABLE               BIT_2
#define UART_C2_TX_ENABLE               BIT_3
#define UART_C2_IDLE_INT_ENABLE         BIT_4
                                  /* BIT_5 depends on C5[RDMAS] to be either: */
#define UART_C2_RX_FULL_INT_ENABLE      BIT_5
#define UART_C2_RX_DMA_TX_ENABLE        BIT_5

#define UART_C2_TX_COMPLETE_INT_ENABLE  BIT_6
                                  /* BIT_5 depends on C5[TDMAS] to be either: */
#define UART_C2_TX_READY_INT_ENABLE     BIT_7
#define UART_C2_TX_DMA_TX_ENABLE        BIT_7


/* S1 */
#define UART_S1_TX_DATA_LOW      BIT_7 /* data <= TWFIFO[TXWATER] */
#define UART_S1_TX_IDLE          BIT_6
#define UART_S1_RX_DATA_FULL     BIT_5 /* data >= RWFIFO[RXWATER] */
#define UART_S1_RX_IDLE          BIT_4
#define UART_S1_RX_OVERRUN       BIT_3
#define UART_S1_RX_NOISE         BIT_2
#define UART_S1_RX_FRAMING_ERROR BIT_1
#define UART_S1_RX_PARITY_ERROR  BIT_0


/* C4 */
#define UART_C4_10_BIT_MODE                 BIT_5
#define UART_C4_MATCH_ADDRESS_MODE_ENABLE_2 BIT_6
#define UART_C4_MATCH_ADDRESS_MODE_ENABLE_1 BIT_7
#define UART_C4_BRFA_MASK 0xf

/*******************************************************************************
* SPI
*******************************************************************************/
typedef enum {
    SPI_MCR_MSTR      =    1 << 31, /* R/W - 0*/
    SPI_MCR_CONT_SCKE =    1 << 30, /* R/W - 0*/
    SPI_MCR_DCONF1    =    1 << 29, /* R/W - 0*/
    SPI_MCR_DCONF0    =    1 << 28, /* R/W - 0*/
    SPI_MCR_DCONF     =  0x3 << 28, /* R/W - 0*/
    SPI_MCR_FRZ       =    1 << 27, /* R/W - 0*/
    SPI_MCR_MTFE      =    1 << 26, /* R/W - 0*/
    SPI_MCR_PCSSE     =    1 << 25, /* R/W - 0*/
    SPI_MCR_ROOE      =    1 << 24, /* R/W - 0*/
    SPI_MCR_PCSIS5    =    1 << 21, /* R/W - 0*/
    SPI_MCR_PCSIS4    =    1 << 20, /* R/W - 0*/
    SPI_MCR_PCSIS3    =    1 << 19, /* R/W - 0*/
    SPI_MCR_PCSIS2    =    1 << 18, /* R/W - 0*/
    SPI_MCR_PCSIS1    =    1 << 17, /* R/W - 0*/
    SPI_MCR_PCSIS0    =    1 << 16, /* R/W - 0*/
    SPI_MCR_PCSIS     = 0x3F << 16, /* R/W - 0*/
    SPI_MCR_DOZE      =    1 << 15, /* R/W - 0*/
    SPI_MCR_MDIS      =    1 << 14, /* R/W - 1*/
    SPI_MCR_DIS_TXF   =    1 << 13, /* R/W - 0*/
    SPI_MCR_DIS_RXF   =    1 << 12, /* R/W - 0*/
    SPI_MCR_CLR_TXF   =    1 << 11, /*   W - 0*/
    SPI_MCR_CLR_RXF   =    1 << 10, /*   W - 0*/
    SPI_MCR_SMPL_PT1  =    1 << 9,  /* R/W - 0*/
    SPI_MCR_SMPL_PT0  =    1 << 8,  /* R/W - 0*/
    SPI_MCR_SMPL_PT   =  0x3 << 8,  /* R/W - 0*/
    SPI_MCR_HALT      =    1 << 0,  /* R/W - 1*/
} spiMcr_t;

typedef enum {
    SPI_CTAR_DBR      =    1 << 31, /* R/W - 0*/
    SPI_CTAR_FMSZ3    =    1 << 30, /* R/W - 1*/
    SPI_CTAR_FMSZ2    =    1 << 29, /* R/W - 1*/
    SPI_CTAR_FMSZ1    =    1 << 28, /* R/W - 1*/
    SPI_CTAR_FMSZ0    =    1 << 27, /* R/W - 1*/
    SPI_CTAR_FMSZ     =  0xF << 27, /* R/W - 1*/
    SPI_CTAR_CPOL     =    1 << 26, /* R/W - 0*/
    SPI_CTAR_CPHA     =    1 << 25, /* R/W - 0*/
    SPI_CTAR_LSBFE    =    1 << 24, /* R/W - 0*/
    SPI_CTAR_PCSSCK1  =    1 << 23, /* R/W - 0*/
    SPI_CTAR_PCSSCK0  =    1 << 22, /* R/W - 0*/
    SPI_CTAR_PCSSCK   =  0x3 << 22, /* R/W - 0*/
    SPI_CTAR_PASC1    =    1 << 21, /* R/W - 0*/
    SPI_CTAR_PASC0    =    1 << 20, /* R/W - 0*/
    SPI_CTAR_PASC     =  0x3 << 20, /* R/W - 0*/
    SPI_CTAR_PDT1     =    1 << 19, /* R/W - 0*/
    SPI_CTAR_PDT0     =    1 << 18, /* R/W - 0*/
    SPI_CTAR_PDT      =  0x3 << 18, /* R/W - 0*/
    SPI_CTAR_PBR1     =    1 << 17, /* R/W - 0*/
    SPI_CTAR_PBR0     =    1 << 16, /* R/W - 0*/
    SPI_CTAR_PBR      =  0x3 << 16, /* R/W - 0*/
    SPI_CTAR_CSSCLK3  =    1 << 15, /* R/W - 0*/
    SPI_CTAR_CSSCLK2  =    1 << 14, /* R/W - 0*/
    SPI_CTAR_CSSCLK1  =    1 << 13, /* R/W - 0*/
    SPI_CTAR_CSSCLK0  =    1 << 12, /* R/W - 0*/
    SPI_CTAR_CSSCLK   =  0xF << 12, /* R/W - 0*/
    SPI_CTAR_ASC3     =    1 << 11, /* R/W - 0*/
    SPI_CTAR_ASC2     =    1 << 10, /* R/W - 0*/
    SPI_CTAR_ASC1     =    1 << 9,  /* R/W - 0*/
    SPI_CTAR_ASC0     =    1 << 8,  /* R/W - 0*/
    SPI_CTAR_ASC      =  0xF << 8,  /* R/W - 0*/
    SPI_CTAR_DT3      =    1 << 7 , /* R/W - 0*/
    SPI_CTAR_DT2      =    1 << 6,  /* R/W - 0*/
    SPI_CTAR_DT1      =    1 << 5,  /* R/W - 0*/
    SPI_CTAR_DT0      =    1 << 4,  /* R/W - 0*/
    SPI_CTAR_DT       =  0xF << 4,  /* R/W - 0*/
    SPI_CTAR_BR3      =    1 << 3 , /* R/W - 0*/
    SPI_CTAR_BR2      =    1 << 2,  /* R/W - 0*/
    SPI_CTAR_BR1      =    1 << 1,  /* R/W - 0*/
    SPI_CTAR_BR0      =    1 << 0,  /* R/W - 0*/
    SPI_CTAR_BR       =  0xF << 0,  /* R/W - 0*/
} spiCtar_t;

typedef enum {
    SPI_PUSHR_CONT     =    1 << 31, /* R/W - 0*/
    SPI_PUSHR_CTAS2    =    1 << 30, /* R/W - 0*/
    SPI_PUSHR_CTAS1    =    1 << 29, /* R/W - 0*/
    SPI_PUSHR_CTAS0    =    1 << 28, /* R/W - 0*/
    SPI_PUSHR_CTAS     =  0x7 << 28, /* R/W - 0*/
    SPI_PUSHR_EOQ      =    1 << 27, /* R/W - 0*/
    SPI_PUSHR_CTCNT    =    1 << 26, /* R/W - 0*/
    SPI_PUSHR_PCS5     =    1 << 21, /* R/W - 0*/
    SPI_PUSHR_PCS4     =    1 << 20, /* R/W - 0*/
    SPI_PUSHR_PCS3     =    1 << 19, /* R/W - 0*/
    SPI_PUSHR_PCS2     =    1 << 18, /* R/W - 0*/
    SPI_PUSHR_PCS1     =    1 << 17, /* R/W - 0*/
    SPI_PUSHR_PCS0     =    1 << 16, /* R/W - 0*/
    SPI_PUSHR_PCS      = 0x3F << 16, /* R/W - 0*/
} spiPushr_t;

typedef enum {
    SPI_RSER_TCF_RE   =    1 << 31, /* R/W - 0*/
    SPI_RSER_EOQF_RE  =    1 << 28, /* R/W - 0*/
    SPI_RSER_TFUF_RE  =    1 << 27, /* R/W - 0*/
    SPI_RSER_TFFF_RE  =    1 << 25, /* R/W - 0*/
    SPI_RSER_TFFF_DIRS=    1 << 24, /* R/W - 0*/
    SPI_RSER_RFOF_RE  =    1 << 19, /* R/W - 0*/
    SPI_RSER_RFDF_RE  =    1 << 17, /* R/W - 0*/
    SPI_RSER_RFDF_DIRS=    1 << 16, /* R/W - 0*/
} spiRser_t;

typedef enum {
    SPI_SR_TCF        =    1 << 31, /* R/W - 0*/
    SPI_SR_TXRXS      =    1 << 30, /* R/W - 0*/
    SPI_SR_EOQF       =    1 << 28, /* R/W - 0*/
    SPI_SR_TFUF       =    1 << 27, /* R/W - 0*/
    SPI_SR_TFFF       =    1 << 25, /* R/W - 0*/
    SPI_SR_RFOF       =    1 << 19, /* R/W - 0*/
    SPI_SR_RFDF       =    1 << 17, /* R/W - 0*/
    SPI_SR_TXCTR3     =    1 << 15, /* R/W - 0*/
    SPI_SR_TXCTR2     =    1 << 14, /* R/W - 0*/
    SPI_SR_TXCTR1     =    1 << 13, /* R/W - 0*/
    SPI_SR_TXCTR0     =    1 << 12, /* R/W - 0*/
    SPI_SR_TXCTR      =  0xF << 12, /* R/W - 0*/
    SPI_SR_TXNXTPTR3  =    1 << 11, /* R/W - 0*/
    SPI_SR_TXNXTPTR2  =    1 << 10, /* R/W - 0*/
    SPI_SR_TXNXTPTR1  =    1 << 9,  /* R/W - 0*/
    SPI_SR_TXNXTPTR0  =    1 << 8,  /* R/W - 0*/
    SPI_SR_TXNXTPTR   =  0xF << 8,  /* R/W - 0*/
    SPI_SR_RXCTR3     =    1 << 7,  /* R/W - 0*/
    SPI_SR_RXCTR2     =    1 << 6,  /* R/W - 0*/
    SPI_SR_RXCTR1     =    1 << 5,  /* R/W - 0*/
    SPI_SR_RXCTR0     =    1 << 4,  /* R/W - 0*/
    SPI_SR_RXCTR      =  0xF << 4,  /* R/W - 0*/
    SPI_SR_POPNXTPTR3 =    1 << 3,  /* R/W - 0*/
    SPI_SR_POPNXTPTR2 =    1 << 2,  /* R/W - 0*/
    SPI_SR_POPNXTPTR1 =    1 << 1,  /* R/W - 0*/
    SPI_SR_POPNXTPTR0 =    1 << 0,  /* R/W - 0*/
    SPI_SR_POPNXTPTR  =    1 << 0,  /* R/W - 0*/
} spiSr_t;

/* SPI0 */
#define SPI0_BASE_ADDR 0x4002C000
#define SPI0_MCR   (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x00))
#define SPI0_TCR   (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x08))
#define SPI0_CTAR0 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x0C))
#define SPI0_CTAR1 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x10))
#define SPI0_SR    (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x2C))
#define SPI0_RSER  (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x30))
#define SPI0_PUSHR (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x34))
#define SPI0_POPR  (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x38))
#define SPI0_TXFR0 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x3C))
#define SPI0_TXFR1 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x40))
#define SPI0_TXFR2 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x44))
#define SPI0_TXFR3 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x48))
#define SPI0_RXFR0 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x7C))
#define SPI0_RXFR1 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x80))
#define SPI0_RXFR2 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x84))
#define SPI0_RXFR3 (*(volatile uint32_t *) (SPI0_BASE_ADDR + 0x88))

/* SPI1 */
#define SPI1_BASE_ADDR 0x4002D000
#define SPI1_MCR   (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x00))
#define SPI1_TCR   (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x08))
#define SPI1_CTAR0 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x0C))
#define SPI1_CTAR1 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x10))
#define SPI1_SR    (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x2C))
#define SPI1_RSER  (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x30))
#define SPI1_PUSHR (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x34))
#define SPI1_POPR  (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x38))
#define SPI1_TXFR0 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x3C))
#define SPI1_TXFR1 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x40))
#define SPI1_TXFR2 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x44))
#define SPI1_TXFR3 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x48))
#define SPI1_RXFR0 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x7C))
#define SPI1_RXFR1 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x80))
#define SPI1_RXFR2 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x84))
#define SPI1_RXFR3 (*(volatile uint32_t *) (SPI1_BASE_ADDR + 0x88))

/* SPI2 */
#define SPI2_BASE_ADDR 0x400AC000
#define SPI2_MCR   (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x00))
#define SPI2_TCR   (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x08))
#define SPI2_CTAR0 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x0C))
#define SPI2_CTAR1 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x10))
#define SPI2_SR    (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x2C))
#define SPI2_RSER  (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x30))
#define SPI2_PUSHR (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x34))
#define SPI2_POPR  (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x38))
#define SPI2_TXFR0 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x3C))
#define SPI2_TXFR1 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x40))
#define SPI2_TXFR2 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x44))
#define SPI2_TXFR3 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x48))
#define SPI2_RXFR0 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x7C))
#define SPI2_RXFR1 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x80))
#define SPI2_RXFR2 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x84))
#define SPI2_RXFR3 (*(volatile uint32_t *) (SPI2_BASE_ADDR + 0x88))

#define SPI_MCR(addr)    (*(volatile uint32_t *) (addr + 0x00))
#define SPI_CTAR0(addr)  (*(volatile uint32_t *) (addr + 0x0C))
#define SPI_CTAR1(addr)  (*(volatile uint32_t *) (addr + 0x10))
#define SPI_SR(addr)     (*(volatile uint32_t *) (addr + 0x2C))
#define SPI_RSER(addr)   (*(volatile uint32_t *) (addr + 0x30))
#define SPI_PUSHR(addr)  (*(volatile uint32_t *) (addr + 0x34))
#define SPI_POPR(addr)   (*(volatile uint32_t *) (addr + 0x38))

/******************************************************************************
 * FTFL FLASH
 *****************************************************************************/
typedef enum {
    FTFL_MGSTAT0    = BIT_0, /* R   - 0 */
    FTFL_FPVIOL     = BIT_4, /* R/W - 0 */
    FTFL_ACCERR     = BIT_5, /* R/W - 0 */
    FTFL_RDCOLERR   = BIT_6, /* R/W - 0 */
    FTFL_CCIF       = BIT_7, /* R/W - 0 */
} ftflFSTAT_t;

typedef enum {
    FTFL_EEERDY     = BIT_0, /* R   - 0 */
    FTFL_RAMRDY     = BIT_1, /* R   - 0 */
    FTFL_PFLSH      = BIT_2, /* R   - 0 */
    FTFL_SWAP       = BIT_3, /* R   - 0 */
    FTFL_ERSSUSP    = BIT_4, /* R/W - 0 */
    FTFL_ERSAREQ    = BIT_5, /* R   - 0 */
    FTFL_RDCOLLIE   = BIT_6, /* R/W - 0 */
    FTFL_CCIE       = BIT_7, /* R/W - 0 */
} ftflFCNFG_t;

typedef enum {
    FTFL_SEC        = BIT_1 | BIT_0, /* R   - 0 */
    FTFL_FSLACC     = BIT_3 | BIT_2, /* R   - 0 */
    FTFL_MEEN       = BIT_5 | BIT_4, /* R   - 0 */
    FTFL_KEYEN      = BIT_7 | BIT_6, /* R   - 0 */
} ftflFSEC_t;

typedef enum {
    FTFL_CMD_READ_1S_BLOCK      = 0x00,
    FTFL_CMD_READ_1S_SECT       = 0x01,
    FTFL_CMD_PRGRM_CHECK        = 0x02,
    FTFL_CMD_READ_RESOURCE      = 0x03,
    FTFL_CMD_PRGRM_LONGWORD     = 0x06,
    FTFL_CMD_ERASE_FLASH_BLOCK  = 0x08,
    FTFL_CMD_ERASE_FLASH_SECT   = 0x09,
    FTFL_CMD_PRGRM_SECT         = 0x0B,
    FTFL_CMD_READ_1S_ALL_BLOCKS = 0x40,
    FTFL_CMD_READ_ONCE          = 0x41,
    FTFL_CMD_PRGRM_ONCE         = 0x43,
    FTFL_CMD_ERASE_ALL_BLOCKS   = 0x44,
    FTFL_CMD_VERIFY_BACKDOOR    = 0x45,
    FTFL_CMD_PRGRM_PARTITION    = 0x80,
    FTFL_CMD_SET_FLEXRAM_FN     = 0x81,
} ftflFCMD_t;

#define FTFL_FLASH_SECTOR_SIZE 0x800 /* Bytes */

#define FTFL_BASE_ADDR 0x40020000
#define FTFL_FSTAT  (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x0))
#define FTFL_FCNFG  (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x1))
#define FTFL_FSEC   (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x2))
#define FTFL_FOPT   (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x3))
#define FTFL_FCCOB3 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x4))
#define FTFL_FCCOB2 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x5))
#define FTFL_FCCOB1 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x6))
#define FTFL_FCCOB0 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x7))
#define FTFL_FCCOB7 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x8))
#define FTFL_FCCOB6 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x9))
#define FTFL_FCCOB5 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0xA))
#define FTFL_FCCOB4 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0xB))
#define FTFL_FCCOBB (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0xC))
#define FTFL_FCCOBA (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0xD))
#define FTFL_FCCOB9 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0xE))
#define FTFL_FCCOB8 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0xF))
#define FTFL_FPROT3 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x10))
#define FTFL_FPROT2 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x11))
#define FTFL_FPROT1 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x12))
#define FTFL_FPROT0 (*(volatile uint8_t *) (FTFL_BASE_ADDR + 0x13))
#endif

