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
#define SIM_UART4_ENABLE  BIT_10

#define SIM_SCGC4 (*(volatile uint32_t *) 0x40048034)
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

#define SIM_SCGC5 (*(volatile uint32_t *) 0x40048038)
#define SIM_PORTA_ENABLE BIT_9
#define SIM_PORTB_ENABLE BIT_10
#define SIM_PORTC_ENABLE BIT_11
#define SIM_PORTD_ENABLE BIT_12
#define SIM_PORTE_ENABLE BIT_13


/*******************************************************************************
* PORT CONTROLLER
*******************************************************************************/
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

#define PORT_PCR(port, pin) (*(volatile uint32_t *) (port + (4 * pin)))

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
#define GPIOA_PDOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x00))
#define GPIOA_PSOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x04))
#define GPIOA_PCOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x08))
#define GPIOA_PTOR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x0c))
#define GPIOA_PDIR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x10))
#define GPIOA_PDDR (*(volatile uint32_t *) (GPIOA_BASE_ADDR + 0x14))

#define GPIOB_BASE_ADDR 0x400ff040
#define GPIOB_PDOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x00))
#define GPIOB_PSOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x04))
#define GPIOB_PCOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x08))
#define GPIOB_PTOR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x0c))
#define GPIOB_PDIR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x10))
#define GPIOB_PDDR (*(volatile uint32_t *) (GPIOB_BASE_ADDR + 0x14))

#define GPIOC_BASE_ADDR 0x400ff080
#define GPIOC_PDOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x00))
#define GPIOC_PSOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x04))
#define GPIOC_PCOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x08))
#define GPIOC_PTOR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x0c))
#define GPIOC_PDIR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x10))
#define GPIOC_PDDR (*(volatile uint32_t *) (GPIOC_BASE_ADDR + 0x14))

#define GPIOD_BASE_ADDR 0x400ff0c0
#define GPIOD_PDOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x00))
#define GPIOD_PSOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x04))
#define GPIOD_PCOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x08))
#define GPIOD_PTOR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x0c))
#define GPIOD_PDIR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x10))
#define GPIOD_PDDR (*(volatile uint32_t *) (GPIOD_BASE_ADDR + 0x14))

#define GPIOE_BASE_ADDR 0x400ff100
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
                                                   /* offset */
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
#define UART4_BASE_ADDR 0x4006e000

#define UART0 UART0_BASE_ADDR
#define UART1 UART1_BASE_ADDR
#define UART2 UART2_BASE_ADDR
#define UART3 UART3_BASE_ADDR
#define UART4 UART4_BASE_ADDR

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

/* C4 */
#define UART_C4_10_BIT_MODE                 BIT_5
#define UART_C4_MATCH_ADDRESS_MODE_ENABLE_2 BIT_6
#define UART_C4_MATCH_ADDRESS_MODE_ENABLE_1 BIT_7
#define UART_C4_BRFA_MASK 0xf

#endif

