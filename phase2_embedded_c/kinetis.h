/*******************************************************************************
*
* kinetis.h
*
*******************************************************************************/
#if !defined(KINETIS_H)
#define KINETIS_H

#include "globalDefs.h"

/* TODO Move to CLOCKS section */
#define SYSTEM_CLOCK_HZ  20480000
#define    BUS_CLOCK_HZ  20480000

/*******************************************************************************
* ARM NVIC
*******************************************************************************/
enum {
    _INIT_STACK_POINTER         = 0,
    ISR_RESET                   = 1,
    ISR_NMI                     = 2,
    ISR_HARD_FAULT              = 3,
    ISR_MEMORY_MANAGMENT_FAULT  = 4,
    ISR_BUS_FAULT               = 5,
    ISR_USAGE_FAULT             = 6,
    ISR_RESERVED_7              = 7,
    ISR_RESERVED_8              = 8,
    ISR_RESERVED_9              = 9,
    ISR_RESERVED_10             = 10,
    ISR_SVC                     = 11,
    ISR_DEBUG_MONITOR           = 12,
    ISR_RESERVED_13             = 13,
    ISR_PEND_SV                 = 14,
    ISR_SYSTICK                 = 15,

                            /* Vendor Specific (Freescale) Defined Exceptions */
    ISR_DMA0                    = 16,
    ISR_DMA1                    = 17,
    ISR_DMA2                    = 18,
    ISR_DMA3                    = 19,
    ISR_DMA4                    = 20,
    ISR_DMA5                    = 21,
    ISR_DMA6                    = 22,
    ISR_DMA7                    = 23,
    ISR_DMA8                    = 24,
    ISR_DMA9                    = 25,
    ISR_DMA10                   = 26,
    ISR_DMA11                   = 27,
    ISR_DMA12                   = 28,
    ISR_DMA13                   = 29,
    ISR_DMA14                   = 30,
    ISR_DMA15                   = 31,
    ISR_DMA_ERROR               = 32,
    ISR_MCM                     = 33,
    ISR_FLASH_CMD_COMPLETE      = 34,
    ISR_FLASH_READ_COLLISION    = 35,
    ISR_MODE_CONTROLLER         = 36,
    ISR_LOW_LEAKAGE_WAKEUP      = 37,
    ISR_WATCHDOG                = 38,
    ISR_RANDOM_NUMBER_GENERATOR = 39,
    ISR_I2C0                    = 40,
    ISR_I2C1                    = 41,
    ISR_SPI0                    = 42,
    ISR_SPI1                    = 43,
    ISR_SPI2                    = 44,
    ISR_CAN0_ORED_MSG_BUFFER    = 45,
    ISR_CAN0_BUFFER_OFF         = 46,
    ISR_CAN0_ERROR              = 47,
    ISR_CAN0_TX_WARNING         = 48,
    ISR_CAN0_RX_WARNING         = 49,
    ISR_CAN0_WAKE_UP            = 50,
    ISR_CAN0_IMEU               = 51,
    ISR_CAN0_LOST_RX            = 52,
    ISR_CAN1_ORED_MSG_BUFFER    = 53,
    ISR_CAN1_BUFFER_OFF         = 54,
    ISR_CAN1_ERROR              = 55,
    ISR_CAN1_TX_WARNING         = 56,
    ISR_CAN1_RX_WARNING         = 57,
    ISR_CAN1_WAKE_UP            = 58,
    ISR_CAN1_IMEU               = 59,
    ISR_CAN1_LOST_RX            = 60,
    ISR_UART0_STATUS_SOURCES    = 61,
    ISR_UART0_ERROR_SOURCES     = 62,
    ISR_UART1_STATUS_SOURCES    = 63,
    ISR_UART1_ERROR_SOURCES     = 64,
    ISR_UART2_STATUS_SOURCES    = 65,
    ISR_UART2_ERROR_SOURCES     = 66,
    ISR_UART3_STATUS_SOURCES    = 67,
    ISR_UART3_ERROR_SOURCES     = 68,
    ISR_UART4_STATUS_SOURCES    = 69,
    ISR_UART4_ERROR_SOURCES     = 70,
    ISR_UART5_STATUS_SOURCES    = 71,
    ISR_UART5_ERROR_SOURCES     = 72,
    ISR_ADC0                    = 73,
    ISR_ADC1                    = 74,
    ISR_CMP0                    = 75,
    ISR_CMP1                    = 76,
    ISR_CMP2                    = 77,
    ISR_FTM0                    = 78,
    ISR_FTM1                    = 79,
    ISR_FTM2                    = 80,
    ISR_CMT                     = 81,
    ISR_RTC                     = 82,
    ISR_RESERVED_83             = 83,
    ISR_PIT0                    = 84,
    ISR_PIT1                    = 85,
    ISR_PIT2                    = 86,
    ISR_PIT3                    = 87,
    ISR_PDB                     = 88,
    ISR_USB_OTG                 = 89,
    ISR_USB_CHARGER_DETECT      = 90,
    ISR_ETHERNET_MAC_1588_TIMER = 91,
    ISR_ETHERNET_MAC_TX         = 92,
    ISR_ETHERNET_MAC_RX         = 93,
    ISR_ETHERNET_MAC_ERROR      = 94,
    ISR_I2S                     = 95,
    ISR_SDHC                    = 96,
    ISR_DAC0                    = 97,
    ISR_DAC1                    = 98,
    ISR_TSI                     = 99,
    ISR_MCG                     = 100,
    ISR_LOW_POWER_TIMER         = 101,
    ISR_RESERVED_102            = 102,
    ISR_GPIO_A                  = 103,
    ISR_GPIO_B                  = 104,
    ISR_GPIO_C                  = 105,
    ISR_GPIO_D                  = 106,
    ISR_GPIO_E                  = 107,
    ISR_RESERVED_108            = 108,
    ISR_RESERVED_109            = 109,
    ISR_SOFTWARE                = 110,

    MAX_ISR,
};

#define NVIC_VTOR (*(volatile uint32_t *) 0xE000ED08)    /* Vector Offset Reg */
#define NVIC_VTOR_SET(x) { NVIC_VTOR = ((x) & 0x1FFFFF80); }

#define NVIC_ICTR   (*(volatile uint32_t *) 0xE000E004)

#define NVIC_ISER_BASE_ADDR  0xE000E100
#define NVIC_ISER0  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x00))
#define NVIC_ISER1  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x04))
#define NVIC_ISER2  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x08))
#define NVIC_ISER3  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x0C))
#define NVIC_ISER4  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x10))
#define NVIC_ISER5  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x14))
#define NVIC_ISER6  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x18))
#define NVIC_ISER7  (*(volatile uint32_t *) (NVIC_ISER_BASE_ADDR + 0x1C))

#define NVIC_ICER_BASE_ADDR  0xE000E180
#define NVIC_ICER0  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x00))
#define NVIC_ICER1  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x04))
#define NVIC_ICER2  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x08))
#define NVIC_ICER3  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x0C))
#define NVIC_ICER4  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x10))
#define NVIC_ICER5  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x14))
#define NVIC_ICER6  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x18))
#define NVIC_ICER7  (*(volatile uint32_t *) (NVIC_ICER_BASE_ADDR + 0x1C))

#define NVIC_ISPR_BASE_ADDR  0xE000E200
#define NVIC_ISPR0  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x00))
#define NVIC_ISPR1  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x04))
#define NVIC_ISPR2  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x08))
#define NVIC_ISPR3  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x0C))
#define NVIC_ISPR4  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x10))
#define NVIC_ISPR5  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x14))
#define NVIC_ISPR6  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x18))
#define NVIC_ISPR7  (*(volatile uint32_t *) (NVIC_ISPR_BASE_ADDR + 0x1C))

#define NVIC_ICPR_BASE_ADDR  0xE000E280
#define NVIC_ICPR0  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x00))
#define NVIC_ICPR1  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x04))
#define NVIC_ICPR2  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x08))
#define NVIC_ICPR3  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x0C))
#define NVIC_ICPR4  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x10))
#define NVIC_ICPR5  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x14))
#define NVIC_ICPR6  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x18))
#define NVIC_ICPR7  (*(volatile uint32_t *) (NVIC_ICPR_BASE_ADDR + 0x1C))

#define NVIC_IABR_BASE_ADDR  0xE000E300
#define NVIC_IABR0  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x00))
#define NVIC_IABR1  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x04))
#define NVIC_IABR2  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x08))
#define NVIC_IABR3  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x0C))
#define NVIC_IABR4  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x10))
#define NVIC_IABR5  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x14))
#define NVIC_IABR6  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x18))
#define NVIC_IABR7  (*(volatile uint32_t *) (NVIC_IABR_BASE_ADDR + 0x1C))

/*******************************************************************************
* ARM SYSTEM CONTROL BLOCK
*******************************************************************************/
enum {
    ARM_SHCSR_USGFAULTENA    = BIT_18,
    ARM_SHCSR_BUSFAULTENA    = BIT_17,
    ARM_SHCSR_MEMFAULTENA    = BIT_16,
    ARM_SHCSR_SVCALLPENDED   = BIT_15,
    ARM_SHCSR_BUSFAULTPENDED = BIT_14,
    ARM_SHCSR_MEMFAULTPENDED = BIT_13,
    ARM_SHCSR_USGFAULTPENDED = BIT_12,
    ARM_SHCSR_SYSTICKACT     = BIT_11,
    ARM_SHCSR_PENDSVACT      = BIT_10,
    ARM_SHCSR_MONITORACT     = BIT_8,
    ARM_SHCSR_SVCALLACT      = BIT_7,
    ARM_SHCSR_USGFAULTACT    = BIT_3,
    ARM_SHCSR_BUSFAULTACT    = BIT_1,
    ARM_SHCSR_MEMFAULTACT    = BIT_0,
};
#define ARM_SHCSR (*(volatile uint32_t *) (0xE000ED24))

/*******************************************************************************
* CROSSBAR
*******************************************************************************/
enum {
    CROSSBAR_MASTER_CODE_BUS,
    CROSSBAR_MASTER_SYS_BUS,
    CROSSBAR_MASTER_DMA,
    CROSSBAR_MASTER_ETH,
    CROSSBAR_MASTER_USB,
    CROSSBAR_MASTER_SDHC,

    MAX_CROSSBAR_MASTERS,
};

enum {
    CROSSBAR_SLAVE_FLASH,
    CROSSBAR_SLAVE_SRAM,
    CROSSBAR_SLAVE_PERIPH0,
    CROSSBAR_SLAVE_PERIPH1_GPIO,
    CROSSBAR_SLAVE_FLEXBUS,

    MAX_CROSSBAR_SLAVES,
};

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
#define SIM_SCGC1_ADDR  0x40048028
#define SIM_SCGC1_PTR     (volatile uint32_t *) SIM_SCGC1_ADDR
#define SIM_SCGC1       (*(volatile uint32_t *) SIM_SCGC1_ADDR)
#define SIM_SCGC1_UART5_ENABLE  BIT_11
#define SIM_SCGC1_UART4_ENABLE  BIT_10

#define SIM_SCGC2_ADDR  0x4004802C
#define SIM_SCGC2_PTR     (volatile uint32_t *) SIM_SCGC2_ADDR
#define SIM_SCGC2       (*(volatile uint32_t *) SIM_SCGC2_ADDR)
#define SIM_SCGC2_DAC0_ENABLE  BIT_12

#define SIM_SCGC3_ADDR  0x40048030
#define SIM_SCGC3_PTR     (volatile uint32_t *) SIM_SCGC3_ADDR
#define SIM_SCGC3       (*(volatile uint32_t *) SIM_SCGC3_ADDR)
#define SIM_SCGC3_SPI2_ENABLE  BIT_12

#define SIM_SCGC4_ADDR  0x40048034
#define SIM_SCGC4_PTR     (volatile uint32_t *) SIM_SCGC4_ADDR
#define SIM_SCGC4       (*(volatile uint32_t *) SIM_SCGC4_ADDR)
#define SIM_SCGC4_EWM_ENABLE    BIT_1
#define SIM_SCGC4_CMT_ENABLE    BIT_2
#define SIM_SCGC4_I2C0_ENABLE   BIT_6
#define SIM_SCGC4_I2C1_ENABLE   BIT_7
#define SIM_SCGC4_UART0_ENABLE  BIT_10
#define SIM_SCGC4_UART1_ENABLE  BIT_11
#define SIM_SCGC4_UART2_ENABLE  BIT_12
#define SIM_SCGC4_UART3_ENABLE  BIT_13
#define SIM_SCGC4_USBOTG_ENABLE BIT_18
#define SIM_SCGC4_CMP_ENABLE    BIT_19
#define SIM_SCGC4_VREF_ENABLE   BIT_20
#define SIM_SCGC4_LLWU_ENABLE   BIT_28

#define SIM_SCGC5_ADDR  0x40048038
#define SIM_SCGC5_PTR     (volatile uint32_t *) SIM_SCGC5_ADDR
#define SIM_SCGC5       (*(volatile uint32_t *) SIM_SCGC5_ADDR)
#define SIM_SCGC5_PORTA_ENABLE BIT_9
#define SIM_SCGC5_PORTB_ENABLE BIT_10
#define SIM_SCGC5_PORTC_ENABLE BIT_11
#define SIM_SCGC5_PORTD_ENABLE BIT_12
#define SIM_SCGC5_PORTE_ENABLE BIT_13

#define SIM_SCGC6_ADDR  0x4004803C
#define SIM_SCGC6_PTR     (volatile uint32_t *) SIM_SCGC6_ADDR
#define SIM_SCGC6       (*(volatile uint32_t *) SIM_SCGC6_ADDR)
#define SIM_SCGC6_SPI0_ENABLE  BIT_12
#define SIM_SCGC6_SPI1_ENABLE  BIT_13
#define SIM_SCGC6_CRC_ENABLE   BIT_18
#define SIM_SCGC6_PIT_ENABLE   BIT_23


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


#define UART0_BASE_ADDR 0x4006a000
#define UART0_REG_PTR   (volatile uartPort_t *) UART0_BASE_ADDR
#define UART1_BASE_ADDR 0x4006b000
#define UART1_REG_PTR   (volatile uartPort_t *) UART1_BASE_ADDR
#define UART2_BASE_ADDR 0x4006c000
#define UART2_REG_PTR   (volatile uartPort_t *) UART2_BASE_ADDR
#define UART3_BASE_ADDR 0x4006d000
#define UART3_REG_PTR   (volatile uartPort_t *) UART3_BASE_ADDR
#define UART4_BASE_ADDR 0x400ea000
#define UART4_REG_PTR   (volatile uartPort_t *) UART4_BASE_ADDR
#define UART5_BASE_ADDR 0x400eb000 /* MK60DN512ZVMD10 */
#define UART5_REG_PTR   (volatile uartPort_t *) UART5_BASE_ADDR

#define UART0 UART0_BASE_ADDR
#define UART1 UART1_BASE_ADDR
#define UART2 UART2_BASE_ADDR
#define UART3 UART3_BASE_ADDR
#define UART4 UART4_BASE_ADDR
#define UART5 UART5_BASE_ADDR

/*******************************************************************************
* UART
*******************************************************************************/

/*******************************************************************************
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
} uartPort_t; /* Memmapped registers */


/* BDH */
typedef enum {
    UART_DH_RX_ACTIVE_INT_ENABLE  = BIT_6, /* R/W - 0 */
    UART_BDH_LIN_BREAK_INT_ENABLE = BIT_7, /* R/W - 0 */
} uartBdh_t;
#define UART_BDH_SBR_MASK 0x1f00
#define UART_BDH_SBR_SHIFT 8

/* BDL */
#define UART_BDL_SBR_MASK 0xff


/* C1 */
typedef enum {
    UART_C1_LOOP_BACK             = BIT_7,
    UART_C1_UART_FREEZE_IN_WAIT   = BIT_6,
    UART_C1_SINGLE_WIRE_LOOP_BACK = BIT_5,
    UART_C1_9_BIT_MODE            = BIT_4,
    UART_C1_ADDRESS_MARK_WAKE     = BIT_3,
    UART_C1_IDLE_LINE_AFTER_STOP  = BIT_2,
    UART_C1_PARITY_ENABLE         = BIT_1,
    UART_C1_PARITY_ODD            = BIT_0,
} uartC1_t;



/* C2 */
typedef enum {
                                  /* BIT_7 depends on C5[TDMAS] to be either: */
    UART_C2_TX_READY_INT_ENABLE    = BIT_7,
    UART_C2_TX_DMA_TX_ENABLE       = BIT_7,

    UART_C2_TX_COMPLETE_INT_ENABLE = BIT_6,
                                  /* BIT_5 depends on C5[RDMAS] to be either: */
    UART_C2_RX_FULL_INT_ENABLE    =  BIT_5,
    UART_C2_RX_DMA_TX_ENABLE      =  BIT_5,

    UART_C2_IDLE_INT_ENABLE       =  BIT_4,
    UART_C2_TX_ENABLE             =  BIT_3,
    UART_C2_RX_ENABLE             =  BIT_2,
    UART_C2_RX_WAKEUP             =  BIT_1,
    UART_C2_SEND_BREAK            =  BIT_0,
} uartC2_t;


/* S1 */
typedef enum {
UART_S1_TX_DATA_LOW      = BIT_7, /* data <= TWFIFO[TXWATER] */
UART_S1_TX_IDLE          = BIT_6,
UART_S1_RX_DATA_FULL     = BIT_5, /* data >= RWFIFO[RXWATER] */
UART_S1_RX_IDLE          = BIT_4,
UART_S1_RX_OVERRUN       = BIT_3,
UART_S1_RX_NOISE         = BIT_2,
UART_S1_RX_FRAMING_ERROR = BIT_1,
UART_S1_RX_PARITY_ERROR  = BIT_0,
} uartS1_t;


/* C4 */
typedef enum {
    UART_C4_MATCH_ADDRESS_MODE_ENABLE_1 = BIT_7,
    UART_C4_MATCH_ADDRESS_MODE_ENABLE_2 = BIT_6,
    UART_C4_10_BIT_MODE                 = BIT_5,
} uartC4_t;
#define UART_C4_BRFA_MASK 0xf

/* PFIFO */
#define UART_PFIFO_TXFIFOSIZE_SHIFT   4
#define UART_PFIFO_RXFIFOSIZE_SHIFT   0
typedef enum {
    UART_PFIFO_TXFE               = BIT_7,
    UART_PFIFO_TXFIFOSIZE_1       = (0x0 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_TXFIFOSIZE_4       = (0x1 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_TXFIFOSIZE_8       = (0x2 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_TXFIFOSIZE_16      = (0x3 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_TXFIFOSIZE_32      = (0x4 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_TXFIFOSIZE_64      = (0x5 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_TXFIFOSIZE_128     = (0x6 << UART_PFIFO_TXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFE               = BIT_3,
    UART_PFIFO_RXFIFOSIZE_1       = (0x0 << UART_PFIFO_RXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFIFOSIZE_4       = (0x1 << UART_PFIFO_RXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFIFOSIZE_8       = (0x2 << UART_PFIFO_RXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFIFOSIZE_16      = (0x3 << UART_PFIFO_RXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFIFOSIZE_32      = (0x4 << UART_PFIFO_RXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFIFOSIZE_64      = (0x5 << UART_PFIFO_RXFIFOSIZE_SHIFT),
    UART_PFIFO_RXFIFOSIZE_128     = (0x6 << UART_PFIFO_RXFIFOSIZE_SHIFT),
} uartPfifo_t;

/* CFIFO */
typedef enum {
    UART_CFIFO_TXFLUSH     = BIT_7,
    UART_CFIFO_RXFLUSH     = BIT_6,
    UART_CFIFO_TXOFE       = BIT_1,
    UART_CFIFO_RXOFE       = BIT_0,
} uartCfifo_t;

/* SFIFO */
typedef enum {
    UART_SFIFO_TXEMPT   = BIT_7,
    UART_SFIFO_RXEMPT   = BIT_6,
    UART_SFIFO_TXOF     = BIT_1,
    UART_SFIFO_RXOF     = BIT_0,
} uartSfifo_t;


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

/* Flash Memory Controller */
enum {
    FMC_CINV_WAY = BIT_20 | BIT_21 | BIT_22 | BIT_23, /* W - 0 */
    FMC_S_B_INV  = BIT_19,                            /* W - 0 */
} fmcPFBCR_t;

#define FMC_BASE_ADDR 0x4001F000
#define FMC_PFAPR   (*(volatile uint32_t *) (FMC_BASE_ADDR + 0x0))
#define FMC_PFB0CR  (*(volatile uint32_t *) (FMC_BASE_ADDR + 0x4))
#define FMC_PFB1CR  (*(volatile uint32_t *) (FMC_BASE_ADDR + 0x8))


/*******************************************************************************
* CRC
*******************************************************************************/
typedef enum {
    CRC_CTRL_TOT1     =    1 << 31, /* R/W - 0*/
    CRC_CTRL_TOT0     =    1 << 30, /* R/W - 0*/
    CRC_CTRL_TOT      =  0x3 << 30, /* R/W - 0*/
    CRC_CTRL_TOTR1    =    1 << 29, /* R/W - 0*/
    CRC_CTRL_TOTR0    =    1 << 28, /* R/W - 0*/
    CRC_CTRL_TOTR     =  0x3 << 28, /* R/W - 0*/
    CRC_CTRL_FXOR     =    1 << 26, /* R/W - 0*/
    CRC_CTRL_WAS      =    1 << 25, /* R/W - 0*/
    CRC_CTRL_TCRC     =    1 << 24, /* R/W - 0*/
} crcCtrl_t;

#define CRC_CRC_ADDR     0x40032000
#define CRC_CRC_PTR      (volatile uint32_t *) CRC_CRC_ADDR
#define CRC_CRC          (*(volatile uint32_t *) CRC_CRC_ADDR)
#define CRC_GPOLY_ADDR   0x40032004
#define CRC_GPOLY32_PTR  (volatile uint32_t *) CRC_GPOLY_ADDR
#define CRC_GPOLY32      (*(volatile uint32_t *) CRC_GPOLY_ADDR)
#define CRC_GPOLY16_PTR  (volatile uint16_t *) CRC_GPOLY_ADDR
#define CRC_GPOLY16      (*(volatile uint16_t *) CRC_GPOLY_ADDR)
#define CRC_CTRL_ADDR    0x40032008
#define CRC_CTRL_PTR     (volatile uint32_t *) CRC_CTRL_ADDR
#define CRC_CTRL         (*(volatile uint32_t *) CRC_CTRL_ADDR)

/*******************************************************************************
* MPU
*******************************************************************************/

enum {
    MPU_VLD     = BIT_0,                             /* R/W - 1 */
    MPU_NGRD    = BIT_8  | BIT_9  | BIT_10 | BIT_11, /* R   - 1 */
    MPU_NSP     = BIT_12 | BIT_13 | BIT_14 | BIT_15, /* R   - 5 */
    MPU_HRL     = BIT_16 | BIT_17 | BIT_18 | BIT_19, /* R   - 1 */
    MPU_SPERR4  = BIT_27,                            /* R   - 0 */
    MPU_SPERR3  = BIT_28,                            /* R   - 0 */
    MPU_SPERR2  = BIT_29,                            /* R   - 0 */
    MPU_SPERR1  = BIT_30,                            /* R   - 0 */
    MPU_SPERR0  = BIT_31,                            /* R   - 0 */
} mpuCESR_t;

#define MPU_EDR_ERW_SHIFT   0
#define MPU_EDR_ERW_MASK    0x1
#define MPU_EDR_EATTR_SHIFT 1
#define MPU_EDR_EATTR_MASK  0x7
#define MPU_EDR_EMN_SHIFT   4
#define MPU_EDR_EMN_MASK    0xF
#define MPU_EDR_EACD_SHIFT  16
#define MPU_EDR_EACD_MASK   0xFFFF

enum {
    MPU_SV_ACCESS_RWX,
    MPU_SV_ACCESS_RX,
    MPU_SV_ACESSS_RW,
    MPU_SV_ACCESS_USER,
};
#define MPU_RGD_M0SM_SHIFT 3
#define MPU_RGD_M1SM_SHIFT 9
#define MPU_RGD_M2SM_SHIFT 15
#define MPU_RGD_M3SM_SHIFT 21

enum {
    MPU_USR_ACCESS_X = BIT_0,
    MPU_USR_ACCESS_W = BIT_1,
    MPU_USR_ACCESS_R = BIT_2,
};
#define MPU_RGD_M0UM_SHIFT  0
#define MPU_RGD_M1UM_SHIFT  6
#define MPU_RGD_M2UM_SHIFT  12
#define MPU_RGD_M3UM_SHIFT  18
#define MPU_RGD_M4UM_SHIFT (24 - 1) /* No eXecute access */
#define MPU_RGD_M5UM_SHIFT (26 - 1) /* No eXecute access */
#define MPU_RGD_M6UM_SHIFT (28 - 1) /* No eXecute access */
#define MPU_RGD_M7UM_SHIFT (30 - 1) /* No eXecute access */

#define MPU_RGD_VALID        BIT_0
#define MPU_RGD_ADDRESS_MASK 0xFFFFFFE0
typedef struct {
    uint32_t word0;
    uint32_t word1;
    uint32_t word2;
    uint32_t word3;
} mpuRGD_t;
#define MAX_MPU_REGIONS 12

#define MPU_BASE_ADDR 0x4000D000
#define MPU_CESR      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x00))

#define MPU_EAR0      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x10))
#define MPU_EDR0      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x14))
#define MPU_EAR1      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x18))
#define MPU_EDR1      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x1C))
#define MPU_EAR2      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x20))
#define MPU_EDR2      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x24))
#define MPU_EAR3      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x28))
#define MPU_EDR3      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x2C))
#define MPU_EAR4      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x30))
#define MPU_EDR4      (*(volatile uint32_t *) (MPU_BASE_ADDR + 0x34))

#define MPU_RGD0_BASE_ADDR  (MPU_BASE_ADDR + 0x400)
#define MPU_RGD1_BASE_ADDR  (MPU_BASE_ADDR + 0x410)
#define MPU_RGD2_BASE_ADDR  (MPU_BASE_ADDR + 0x420)
#define MPU_RGD3_BASE_ADDR  (MPU_BASE_ADDR + 0x430)
#define MPU_RGD4_BASE_ADDR  (MPU_BASE_ADDR + 0x440)
#define MPU_RGD5_BASE_ADDR  (MPU_BASE_ADDR + 0x450)
#define MPU_RGD6_BASE_ADDR  (MPU_BASE_ADDR + 0x460)
#define MPU_RGD7_BASE_ADDR  (MPU_BASE_ADDR + 0x470)
#define MPU_RGD8_BASE_ADDR  (MPU_BASE_ADDR + 0x480)
#define MPU_RGD9_BASE_ADDR  (MPU_BASE_ADDR + 0x490)
#define MPU_RGD10_BASE_ADDR (MPU_BASE_ADDR + 0x4A0)
#define MPU_RGD11_BASE_ADDR (MPU_BASE_ADDR + 0x4B0)

#define MPU_RGDAAC_BASE_ADDR (MPU_BASE_ADDR + 0x800)
#define MPU_RGDAAC0  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x00))
#define MPU_RGDAAC1  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x04))
#define MPU_RGDAAC2  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x08))
#define MPU_RGDAAC3  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x0C))
#define MPU_RGDAAC4  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x10))
#define MPU_RGDAAC5  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x14))
#define MPU_RGDAAC6  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x18))
#define MPU_RGDAAC7  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x1C))
#define MPU_RGDAAC8  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x20))
#define MPU_RGDAAC9  (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x24))
#define MPU_RGDAAC10 (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x28))
#define MPU_RGDAAC11 (*(volatile uint32_t *) (MPU_RGDAAC_BASE_ADDR + 0x2C))

/*******************************************************************************
* DAC
*******************************************************************************/

#define DAC0_BASE_ADDR 0x400CC000

#define DAC0_DAT0L_ADDR (DAC0_BASE_ADDR + 0x0)
#define DAC0_DAT0H_ADDR (DAC0_BASE_ADDR + 0x1)
#define DAC0_DAT1L_ADDR (DAC0_BASE_ADDR + 0x2)
#define DAC0_DAT1H_ADDR (DAC0_BASE_ADDR + 0x3)

#define DAC0_DAT (*(volatile uint8_t *) (DAC0_BASE_ADDR)

#define DAC0_SR  (*(volatile uint8_t *) (DAC0_BASE_ADDR + 0x20)
#define DAC0_CR0 (*(volatile uint8_t *) (DAC0_BASE_ADDR + 0x21)
#define DAC0_CR1 (*(volatile uint8_t *) (DAC0_BASE_ADDR + 0x22)
#define DAC0_CR2 (*(volatile uint8_t *) (DAC0_BASE_ADDR + 0x23)

#define DAC_DATA_MAX 16

#define DAC_SR_BFWMF  BIT_2  /* watermark flag */
#define DAC_SR_BFRPTF BIT_1  /* read pointer top position flag */
#define DAC_SR_BFRPBF BIT_0  /* read pointer bottom position flag */

#define DAC_CR0_DACEN     BIT_7 /* Enable */
#define DAC_CR0_DACRFS    BIT_6 /* Reference select, 0 = DACREF_1
			    	                     1 = DACREF_2 */
#define DAC_CR0_DACTRGSEL BIT_5 /* Trigger select, 0 = HW trigger
				                   1 = SW trigger */
#define DAC_CR0_DACSWTRG  BIT_4 /* Software trigger, active high, WO */
#define DAC_CR0_LPEN      BIT_3 /* Low power control, 0 = high power
				                      1 = low power */
#define DAC_CR0_DACBWIEN  BIT_2 /* Buffer watermark interrupt enable */
#define DAC_CR0_DACBTIEN  BIT_1 /* Buffer read pointer top flag enable */
#define DAC_CR0_DACBBIEN  BIT_0 /* Buffer read pointer bottom flag enable */

#define DAC_CR1_DMAEN     BIT_7 /* DMA Enabled */

enum {
    DAC_MODE_NORMAL,
    DAC_MODE_SWING,
    DAC_MODE_ONE_TIME,
    DAC_MODE_RESERVED
};

#define DAC_CR0_FLAG (DAC_CR0_DACEN | DAC_CR0_DACRFS | DAC_CR0_DACTRGSEL)
#define DAC_CR0_TRGF (DAC_CR0_FLAG | DAC_CR0_DACSWTRG)
#define DAC_CR1_FLAG 0x01
#define DAC_CR2_FLAG 0x0f

/*******************************************************************************
* PIT
*******************************************************************************/

#define PIT_CTRL_BASE 0x40037000
#define PIT_CH_OFFSET 0x010
#define PIT_LDVALN_OFFSET 0x100
#define	PIT_CVALN_OFFSET 0x104
#define PIT_TCTRLN_OFFSET 0x108
#define PIT_TFLGN_OFFSET 0x10C

#define PIT0_MCR (PIT_CTRL_BASE + (PIT_CH_OFFSET * 0x0))
#define PIT0_LDVAL (PIT_CTRL_BASE + PIT_LDVALN_OFFSET)
#define PIT0_CVAL (PIT_CTRL_BASE + PIT_CVALN_OFFSET)
#define PIT0_TCTRL (PIT_CTRL_BASE + PIT_TCTRLN_OFFSET)
#define PIT0_TFLG (PIT_CTRL_BASE + PIT_TFLGN_OFFSET)

#define MCR_MDIS (1 << 1)
#define MCR_FRZ (1 << 0)
#define TCTRL_TIE (1 << 1)
#define TCTRL_TEN (1 << 0)
#define TFLG_TIF (1 << 0)

#define PIT_ENABLE (1 << 23)
#define	SIM_SCGC6_FLAGS (PIT_ENABLE)

#endif
