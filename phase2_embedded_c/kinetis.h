/*******************************************************************************
*
* kinetis.h
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
#if !defined(KINETIS_H)
#define KINETIS_H

#include "globalDefs.h"

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

/* System Options Registers */
#define SIM_SOPT1_ADDR  0x40047000
#define SIM_SOPT1_PTR (volatile uint32_t *) SIM_SOPT1_ADDR
#define SIM_SOPT1   (*(volatile uint32_t *) SIM_SOPT1_ADDR)
#define SIM_SOPT1_OSC32KSEL  BIT_19
#define SIM_SOPT2_ADDR  0x40048004
#define SIM_SOPT2_PTR (volatile uint32_t *) SIM_SOPT2_ADDR
#define SIM_SOPT2   (*(volatile uint32_t *) SIM_SOPT2_ADDR)
#define SIM_SOPT2_PLLFLLSEL  BIT_16
#define SIM_SOPT2_MCGCLKSEL  BIT_0


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
#define SIM_SCGC3_ADC1_ENABLE  BIT_27

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

#define SIM_TSI_ENABLE   BIT_5

#define SIM_SCGC6_ADDR  0x4004803C
#define SIM_SCGC6_PTR     (volatile uint32_t *) SIM_SCGC6_ADDR
#define SIM_SCGC6       (*(volatile uint32_t *) SIM_SCGC6_ADDR)
#define SIM_SCGC6_SPI0_ENABLE  BIT_12
#define SIM_SCGC6_SPI1_ENABLE  BIT_13
#define SIM_SCGC6_CRC_ENABLE   BIT_18
#define SIM_SCGC6_PIT_ENABLE   BIT_23
#define SIM_SCGC6_ADC0_ENABLE  BIT_27
#define SIM_SCGC6_FTFL_ENABLE  BIT_0
#define SIM_SCGC7_ADDR  0x40048040
#define SIM_SCGC7_PTR (volatile uint32_t *) SIM_SCGC7_ADDR
#define SIM_SCGC7   (*(volatile uint32_t *) SIM_SCGC7_ADDR)
#define SIM_SCGC7_FLEXBUS_ENABLE BIT_0

/* System Clock Divider Registers */
#define SIM_CLKDIV1_ADDR  0x40048044
#define SIM_CLKDIV1_PTR (volatile uint32_t *) SIM_CLKDIV1_ADDR
#define SIM_CLKDIV1   (*(volatile uint32_t *) SIM_CLKDIV1_ADDR)
#define SIM_CLKDIV1_OUTDIV1_MASK  0xF << 28 /* Core/System clock divider */
#define SIM_CLKDIV1_OUTDIV2_MASK  0xF << 24 /* Bus/Peripheral clock divider */
#define SIM_CLKDIV1_OUTDIV3_MASK  0xF << 20 /* FlexBus clock divider */
#define SIM_CLKDIV1_OUTDIV4_MASK  0xF << 16 /* Flash clock divider */


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
* ADC
*******************************************************************************/

#define ADC0_BASE_ADDR 0x4003b000
#define ADC0_REG_PTR   (volatile adc_t *) ADC0_BASE_ADDR
#define ADC1_BASE_ADDR 0x400bb000
#define ADC1_REG_PTR   (volatile adc_t *) ADC1_BASE_ADDR


#define ADC0 UART0_BASE_ADDR
#define ADC1 UART0_BASE_ADDR

typedef struct {
    uint32_t sc1a;    /* Status and control registers 1, R/W, 0x0000001F */
    uint32_t sc1b;    /* Status and control registers 1, R/W, 0x0000001F */
    uint32_t cfg1;    /* Config register 1             , R/W, 0x00000000 */
    uint32_t cfg2;    /* Config register 2             , R/W, 0x00000000 */
    uint32_t ra;      /* Data result register          , R  , 0x00000000 */
    uint32_t rb;      /* Data result register          , R  , 0x00000000 */
    uint32_t cv1;     /* Compare value registers       , R/W, 0x00000000 */
    uint32_t cv2;     /* Compare value registers       , R/W, 0x00000000 */
    uint32_t sc2;     /* Status and control registers 2, R/W, 0x00000000 */
    uint32_t sc3;     /* Status and control registers 3, R/W, 0x00000000 */
    uint32_t ofs;     /* Offset correction register    , R/W, 0x00000004 */
    uint32_t pg;      /* Plus-side gain register       , R/W, 0x00008200 */
    uint32_t mg;      /* Minus-side gain register      , R/W, 0x00008200 */
    uint32_t clpd;    /* Plus-side general cal. value  , R/W, 0x0000000A */
    uint32_t clps;    /* Plus-side general cal. value  , R/W, 0x00000020 */
    uint32_t clp4;    /* Plus-side general cal. value  , R/W, 0x00000200 */
    uint32_t clp3;    /* Plus-side general cal. value  , R/W, 0x00000100 */
    uint32_t clp2;    /* Plus-side general cal. value  , R/W, 0x00000080 */
    uint32_t clp1;    /* Plus-side general cal. value  , R/W, 0x00000040 */
    uint32_t clp0;    /* Plus-side general cal. value  , R/W, 0x00000020 */
    uint32_t pga;     /* PGA register                  , R/W, 0x00000000 */
    uint32_t clmd;    /* Minus-side general cal. value  , R/W, 0x0000000A */
    uint32_t clms;    /* Minus-side general cal. value  , R/W, 0x00000020 */
    uint32_t clm4;    /* Minus-side general cal. value  , R/W, 0x00000200 */
    uint32_t clm3;    /* Minus-side general cal. value  , R/W, 0x00000100 */
    uint32_t clm2;    /* Minus-side general cal. value  , R/W, 0x00000080 */
    uint32_t clm1;    /* Minus-side general cal. value  , R/W, 0x00000040 */
    uint32_t clm0;    /* Minus-side general cal. value  , R/W, 0x00000020 */
} adc_t; /* Memmapped registers */

enum {
    ADC_REGISTERS_A,
    ADC_REGISTERS_B
};

/* SC1n */
typedef enum {
    ADC_SC1_COCO_BIT  = 1 << 7,
    ADC_SC1_AIEN_BIT  = 1 << 6,
    ADC_SC1_DIFF_BIT  = 1 << 5,
} adcSc1_t;
#define ADC_SC1_ADCH_MASK 0x1f

typedef enum {
    ADC_SC1_ADCH_CH0,
    ADC_SC1_ADCH_CH1,
    ADC_SC1_ADCH_CH2,
    ADC_SC1_ADCH_CH3,
    /* CH4 - CH23 valid when DIFF=0, reserved otherwise */
    ADC_SC1_ADCH_CH4,
    ADC_SC1_ADCH_CH5,
    ADC_SC1_ADCH_CH6,
    ADC_SC1_ADCH_CH7,
    ADC_SC1_ADCH_CH8,
    ADC_SC1_ADCH_CH9,
    ADC_SC1_ADCH_CH10,
    ADC_SC1_ADCH_CH11,
    ADC_SC1_ADCH_CH12,
    ADC_SC1_ADCH_CH13,
    ADC_SC1_ADCH_CH14,
    ADC_SC1_ADCH_CH15,
    ADC_SC1_ADCH_CH16,
    ADC_SC1_ADCH_CH17,
    ADC_SC1_ADCH_CH18,
    ADC_SC1_ADCH_CH19,
    ADC_SC1_ADCH_CH20,
    ADC_SC1_ADCH_CH21,
    ADC_SC1_ADCH_CH22,
    ADC_SC1_ADCH_CH23,
    ADC_SC1_ADCH_RESERVED1,
    ADC_SC1_ADCH_RESERVED2,
    ADC_SC1_ADCH_TEMP_SENS,
    ADC_SC1_ADCH_TEMP_BANDGAP,
    ADC_SC1_ADCH_RESERVED3,
    /* Voltage references selected is determined by REFSEL in SC2 */
    ADC_SC1_ADCH_V_REFSH, /* V_REFSH if DIFF = 0, -V_REFSH if DIFF = 1 */
    ADC_SC1_ADCH_V_REFSL, /* V_REFSL if DIFF = 0, Reserved if DIFF = 1 */
    ADC_SC1_ADCH_DISABLED,
} adcSc1ADCH_t;

/* CFG1 */
#define ADC_CFG1_ADLPC_BIT  (1 << 7)
#define ADC_CFG1_ADIV_MASK  0x3
#define ADC_CFG1_ADIV_SHIFT 5
#define ADC_CFG1_ADLSMP_BIT (1 << 4)
#define ADC_CFG1_MODE_MASK  0x3
#define ADC_CFG1_MODE_SHIFT 2
#define ADC_CFG1_ADICLK_MASK  0x3

enum {
    ADC_CFG1_ADIV_1,
    ADC_CFG1_ADIV_2,
    ADC_CFG1_ADIV_4,
    ADC_CFG1_ADIV_8,
};

enum {
    ADC_CFG1_MODE_8_BIT,  /* 2's complitment 9-bit output in DIFF */
    ADC_CFG1_MODE_12_BIT, /* 2's complitment 13-bit output in DIFF */
    ADC_CFG1_MODE_10_BIT, /* 2's complitment 11-bit output in DIFF */
    ADC_CFG1_MODE_16_BIT, /* 2's complitment 16-bit output in DIFF */
};

enum {
    ADC_CFG1_ADICLK_BUS,
    ADC_CFG1_ADICLK_BUS_DIV_2,
    ADC_CFG1_ADICLK_ALTCLK,
    ADC_CFG1_ADICLK_ADACK,
};

/* CFG2 */
enum {
    ADC_CFG2_MUXSEL_BIT  = 1 << 4, /* Selects ADxxb */
    ADC_CFG2_ADACKEN_BIT = 1 << 3,
    ADC_CFG2_ADHSC_BIT   = 1 << 2,
};

enum {
    ADC_CFG2_ADLSTS_ADCK_20,
    ADC_CFG2_ADLSTS_ADCK_12,
    ADC_CFG2_ADLSTS_ADCK_6,
    ADC_CFG2_ADLSTS_ADCK_2,
};
#define ADC_CFG_ADLSTS_MASK 0x3

/* CVn */
#define ADC_CVN_CV_MASK 0xFFFF

/* SC2 */
enum {
    ADC_SC2_ADACT_BIT   = 1 << 7,
    ADC_SC2_ADTRG_BIT   = 1 << 6, /* Set for hardware trigger */
    ADC_SC2_ACFE_BIT    = 1 << 5, /* Compare function enable */
    ADC_SC2_ACFGT_BIT   = 1 << 4, /* Compare function greater than enable */
    ADC_SC2_ADREN_BIT   = 1 << 3, /* Compare function range enable */
    ADC_SC2_DMAEN_BIT   = 1 << 2, /* DMA enable */
};

enum {
    ADC_SC2_REFESL_DEFAULT,   /* External pins V_REFH, V_REFL */
    ADC_SC2_REFESL_ALTERNATE, /* V_ALTH, V_ALTL */
};
#define ADC_SC2_REFESL_MASK 0x3

/* SC3 */
enum {
    ADC_SC3_CAL_BIT     = 1 << 7,
    ADC_SC3_CALF_BIT    = 1 << 6,

    ADC_SC3_ADCO_BIT    = 1 << 3, /* Enable continuous conversions */
    ADC_SC3_AVGE_BIT    = 1 << 2, /* Enable hardware average function */
};

enum {
    ADC_SC3_AVGS_4,
    ADC_SC3_AVGS_8,
    ADC_SC3_AVGS_16,
    ADC_SC3_AVGS_32,
};
#define ADC_SC3_AVGS_MASK 0x3


/* OFS, PG, MG */
#define ADC_OFS_MASK 0xFFFF
#define ADC_PG_MASK  0xFFFF
#define ADC_MG_MASK  0xFFFF

/* CLP* */
#define ADC_CLPD_MASK  0x3F
#define ADC_CLPS_MASK  0x3F
#define ADC_CLP4_MASK  0x3FF
#define ADC_CLP3_MASK  0x1FF
#define ADC_CLP2_MASK  0xFF
#define ADC_CLP1_MASK  0x7F
#define ADC_CLP0_MASK  0x3F

/*  PGA */
enum {
    ADC_PGA_PGAEN_BIT    = 1 << 23,
    ADC_PGA_PGALPB_BIT   = 1 << 20,
};

enum {
    ADC_PGA_PGAG_1,
    ADC_PGA_PGAG_2,
    ADC_PGA_PGAG_4,
    ADC_PGA_PGAG_8,
    ADC_PGA_PGAG_16,
    ADC_PGA_PGAG_32,
    ADC_PGA_PGAG_64,
};
#define ADC_PGA_PGAG_MASK 0xF
#define ADC_PGA_PGAG_SHIFT 16


/* CLM* */
#define ADC_CLMD_MASK  0x3F
#define ADC_CLMS_MASK  0x3F
#define ADC_CLM4_MASK  0x3FF
#define ADC_CLM3_MASK  0x1FF
#define ADC_CLM2_MASK  0xFF
#define ADC_CLM1_MASK  0x7F
#define ADC_CLM0_MASK  0x3F




/*******************************************************************************
* UART
*******************************************************************************/

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
UART_S1_TX_DATA_LOW      = BIT_7, /* TDRE data <= TWFIFO[TXWATER] */
UART_S1_TX_IDLE          = BIT_6, /* TC */
UART_S1_RX_DATA_FULL     = BIT_5, /* RDRF data >= RWFIFO[RXWATER] */
UART_S1_RX_IDLE          = BIT_4, /* IDLE */
UART_S1_RX_OVERRUN       = BIT_3, /* OR */
UART_S1_RX_NOISE         = BIT_2, /* NF */
UART_S1_RX_FRAMING_ERROR = BIT_1, /* FE */
UART_S1_RX_PARITY_ERROR  = BIT_0, /* PF */
} uartS1_t;


/* C4 */
typedef enum {
    UART_C4_MATCH_ADDRESS_MODE_ENABLE_1 = BIT_7,
    UART_C4_MATCH_ADDRESS_MODE_ENABLE_2 = BIT_6,
    UART_C4_10_BIT_MODE                 = BIT_5,
} uartC4_t;
#define UART_C4_BRFA_MASK 0xf

/* PFIFO */
#define UART_TXFIFOSIZE_MASK  0x70
#define UART_RXFIFOSIZE_MASK  0x07
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
* TSI
*******************************************************************************/
#define TSI0_BASE_ADDR 0x40045000
#define TSI_GENCS(addr)       (*(volatile uint32_t *) (addr + 0x00))
#define TSI_SCANC(addr)       (*(volatile uint32_t *) (addr + 0x04))
#define TSI_PEN(addr)         (*(volatile uint32_t *) (addr + 0x08))
#define TSI_STATUS(addr)      (*(volatile uint32_t *) (addr + 0x0C))
#define TSI_CNTR1(addr)       (*(volatile uint32_t *) (addr + 0x100))
#define TSI_CNTR3(addr)       (*(volatile uint32_t *) (addr + 0x104))
#define TSI_CNTR5(addr)       (*(volatile uint32_t *) (addr + 0x108))
#define TSI_CNTR7(addr)       (*(volatile uint32_t *) (addr + 0x10C))
#define TSI_CNTR9(addr)       (*(volatile uint32_t *) (addr + 0x110))
#define TSI_CNTR11(addr)      (*(volatile uint32_t *) (addr + 0x114))
#define TSI_CNTR13(addr)      (*(volatile uint32_t *) (addr + 0x118))
#define TSI_CNTR15(addr)      (*(volatile uint32_t *) (addr + 0x11C))
#define TSI_CNTR(addr)        ( (volatile uint16_t *) (addr + 0x100))
#define TSI_THRESHOLD0(addr)  (*(volatile uint32_t *) (addr + 0x120))
#define TSI_THRESHOLD1(addr)  (*(volatile uint32_t *) (addr + 0x124))
#define TSI_THRESHOLD2(addr)  (*(volatile uint32_t *) (addr + 0x128))
#define TSI_THRESHOLD3(addr)  (*(volatile uint32_t *) (addr + 0x12C))
#define TSI_THRESHOLD4(addr)  (*(volatile uint32_t *) (addr + 0x130))
#define TSI_THRESHOLD5(addr)  (*(volatile uint32_t *) (addr + 0x134))
#define TSI_THRESHOLD6(addr)  (*(volatile uint32_t *) (addr + 0x138))
#define TSI_THRESHOLD7(addr)  (*(volatile uint32_t *) (addr + 0x13C))
#define TSI_THRESHOLD8(addr)  (*(volatile uint32_t *) (addr + 0x140))
#define TSI_THRESHOLD9(addr)  (*(volatile uint32_t *) (addr + 0x144))
#define TSI_THRESHOLD10(addr) (*(volatile uint32_t *) (addr + 0x148))
#define TSI_THRESHOLD11(addr) (*(volatile uint32_t *) (addr + 0x14C))
#define TSI_THRESHOLD12(addr) (*(volatile uint32_t *) (addr + 0x150))
#define TSI_THRESHOLD13(addr) (*(volatile uint32_t *) (addr + 0x154))
#define TSI_THRESHOLD14(addr) (*(volatile uint32_t *) (addr + 0x158))
#define TSI_THRESHOLD15(addr) (*(volatile uint32_t *) (addr + 0x15C))
#define TSI_THRESHOLD(addr)   ( (volatile uint32_t *) (addr + 0x120))

#define TSI0_GENCS            (TSI_GENCS      (TSI0_BASE_ADDR))
#define TSI0_SCANC            (TSI_SCANC      (TSI0_BASE_ADDR))
#define TSI0_PEN              (TSI_PEN        (TSI0_BASE_ADDR))
#define TSI0_STATUS           (TSI_STATUS     (TSI0_BASE_ADDR))
#define TSI0_CNTR1            (TSI_CNTR1      (TSI0_BASE_ADDR))
#define TSI0_CNTR3            (TSI_CNTR3      (TSI0_BASE_ADDR))
#define TSI0_CNTR5            (TSI_CNTR5      (TSI0_BASE_ADDR))
#define TSI0_CNTR7            (TSI_CNTR7      (TSI0_BASE_ADDR))
#define TSI0_CNTR9            (TSI_CNTR9      (TSI0_BASE_ADDR))
#define TSI0_CNTR11           (TSI_CNTR11     (TSI0_BASE_ADDR))
#define TSI0_CNTR13           (TSI_CNTR13     (TSI0_BASE_ADDR))
#define TSI0_CNTR15           (TSI_CNTR15     (TSI0_BASE_ADDR))
#define TSI0_CNTR             (TSI_CNTR       (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD0       (TSI_THRESHOLD0 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD1       (TSI_THRESHOLD1 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD2       (TSI_THRESHOLD2 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD3       (TSI_THRESHOLD3 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD4       (TSI_THRESHOLD4 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD5       (TSI_THRESHOLD5 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD6       (TSI_THRESHOLD6 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD7       (TSI_THRESHOLD7 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD8       (TSI_THRESHOLD8 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD9       (TSI_THRESHOLD9 (TSI0_BASE_ADDR))
#define TSI0_THRESHOLD10      (TSI_THRESHOLD10(TSI0_BASE_ADDR))
#define TSI0_THRESHOLD11      (TSI_THRESHOLD11(TSI0_BASE_ADDR))
#define TSI0_THRESHOLD12      (TSI_THRESHOLD12(TSI0_BASE_ADDR))
#define TSI0_THRESHOLD13      (TSI_THRESHOLD13(TSI0_BASE_ADDR))
#define TSI0_THRESHOLD14      (TSI_THRESHOLD14(TSI0_BASE_ADDR))
#define TSI0_THRESHOLD15      (TSI_THRESHOLD15(TSI0_BASE_ADDR))
#define TSI0_THRESHOLD        (TSI_THRESHOLD  (TSI0_BASE_ADDR))

typedef enum {
    TSI_GENCS_LPCLKS          = 1 << 28,    /* R/W - 0 */
    TSI_GENCS_LPSCNITV_MASK   = 0xF << 24,  /* R/W - 0 */
    TSI_GENCS_LPSCNITV_1MS    = 0x0 << 24,
    TSI_GENCS_LPSCNITV_5MS    = 0x1 << 24,
    TSI_GENCS_LPSCNITV_10MS   = 0x2 << 24,
    TSI_GENCS_LPSCNITV_15MS   = 0x3 << 24,
    TSI_GENCS_LPSCNITV_20MS   = 0x4 << 24,
    TSI_GENCS_LPSCNITV_30MS   = 0x5 << 24,
    TSI_GENCS_LPSCNITV_40MS   = 0x6 << 24,
    TSI_GENCS_LPSCNITV_50MS   = 0x7 << 24,
    TSI_GENCS_LPSCNITV_75MS   = 0x8 << 24,
    TSI_GENCS_LPSCNITV_100MS  = 0x9 << 24,
    TSI_GENCS_LPSCNITV_125MS  = 0xA << 24,
    TSI_GENCS_LPSCNITV_150MS  = 0xB << 24,
    TSI_GENCS_LPSCNITV_200MS  = 0xC << 24,
    TSI_GENCS_LPSCNITV_300MS  = 0xD << 24,
    TSI_GENCS_LPSCNITV_400MS  = 0xE << 24,
    TSI_GENCS_LPSCNITV_500MS  = 0xF << 24,
    TSI_GENCS_NSCN_MASK       = 0x1F << 19, /* R/W - 0 */
    TSI_GENCS_NSCN_SHIFT      = 19,
    TSI_GENCS_PS_MASK         = 0x7 << 16,  /* R/W - 0 */
    TSI_GENCS_PS_SHIFT        = 16,
    TSI_GENCS_EOSF            = 1 << 15,    /* R/W - 0 */
    TSI_GENCS_OUTRGF          = 1 << 14,    /* R/W - 0 */
    TSI_GENCS_EXTERF          = 1 << 13,    /* R/W - 0 */
    TSI_GENCS_OVRF            = 1 << 12,    /* R/W - 0 */
    TSI_GENCS_SCNIP           = 1 << 9,     /* R/W - 0 */
    TSI_GENCS_SWTS            = 1 << 8,     /* R/W - 0 */
    TSI_GENCS_TSIEN           = 1 << 7,     /* R/W - 0 */
    TSI_GENCS_TSIIE           = 1 << 6,     /* R/W - 0 */
    TSI_GENCS_ERIE            = 1 << 5,     /* R/W - 0 */
    TSI_GENCS_ESOR            = 1 << 4,     /* R/W - 0 */
    TSI_GENCS_STM             = 1 << 1,     /* R/W - 0 */
    TSI_GENCS_STPE            = 1 << 0,     /* R/W - 0 */
} tsiGencs_t;

typedef enum {
    TSI_SCANC_REFCHRG_MASK    = 0x1F << 27, /* R/W - 0 */
    TSI_SCANC_REFCHRG_SHIFT   = 27,
    TSI_SCANC_CAPTRM_MASK     = 0x7 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_SHIFT    = 24,
    TSI_SCANC_CAPTRM_0p5      = 0x0 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_0p6      = 0x1 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_0p7      = 0x2 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_0p8      = 0x3 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_0p9      = 0x4 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_1p0      = 0x5 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_1p1      = 0x6 << 24,  /* R/W - 0 */
    TSI_SCANC_CAPTRM_1p2      = 0x7 << 24,  /* R/W - 0 */
    TSI_SCANC_EXTCHRG_MASK    = 0x1F << 19, /* R/W - 0 */
    TSI_SCANC_EXTCHRG_SHIFT   = 19,
    TSI_SCANC_DELVOL_MASK     = 0x7 << 16,  /* R/W - 0 */
    TSI_SCANC_DELVOL_100mV    = 0x0 << 16,
    TSI_SCANC_DELVOL_150mV    = 0x1 << 16,
    TSI_SCANC_DELVOL_200mV    = 0x2 << 16,
    TSI_SCANC_DELVOL_250mV    = 0x3 << 16,
    TSI_SCANC_DELVOL_300mV    = 0x4 << 16,
    TSI_SCANC_DELVOL_400mV    = 0x5 << 16,
    TSI_SCANC_DELVOL_500mV    = 0x6 << 16,
    TSI_SCANC_DELVOL_600mV    = 0x7 << 16,
    TSI_SCANC_SMOD_MASK       = 0xFF << 8, /* R/W - 0 */
    TSI_SCANC_SMOD_SHIFT      = 8,
    TSI_SCANC_AMCLKDIV        = 1 << 5,    /* R/W - 0 */
    TSI_SCANC_AMCLKS_MASk     = 3 << 3,    /* R/W - 0 */
    TSI_SCANC_AMCLKS_BUS_CLK  = 0 << 3,
    TSI_SCANC_AMCLKS_MCGIRCLK = 1 << 3,
    TSI_SCANC_AMCLKS_OSCERCLK = 2 << 3,
    TSI_SCANC_AMPSC_MASK      = 7 << 0,    /* R/W - 0 */
    TSI_SCANC_AMPSC_SHIFT     = 0,
} tsiScanc_t;

typedef enum {
    TSI_PEN_LPSCP_MASK        = 0xF << 16, /* R/W - 0 */
    TSI_PEN_LPSCP_SHIFT       = 16,
    TSI_PEN_PEN15             = 1 << 15,   /* R/W - 0 */
    TSI_PEN_PEN14             = 1 << 14,   /* R/W - 0 */
    TSI_PEN_PEN13             = 1 << 13,   /* R/W - 0 */
    TSI_PEN_PEN12             = 1 << 12,   /* R/W - 0 */
    TSI_PEN_PEN11             = 1 << 11,   /* R/W - 0 */
    TSI_PEN_PEN10             = 1 << 10,   /* R/W - 0 */
    TSI_PEN_PEN9              = 1 << 9,    /* R/W - 0 */
    TSI_PEN_PEN8              = 1 << 8,    /* R/W - 0 */
    TSI_PEN_PEN7              = 1 << 7,    /* R/W - 0 */
    TSI_PEN_PEN6              = 1 << 6,    /* R/W - 0 */
    TSI_PEN_PEN5              = 1 << 5,    /* R/W - 0 */
    TSI_PEN_PEN4              = 1 << 4,    /* R/W - 0 */
    TSI_PEN_PEN3              = 1 << 3,    /* R/W - 0 */
    TSI_PEN_PEN2              = 1 << 2,    /* R/W - 0 */
    TSI_PEN_PEN1              = 1 << 1,    /* R/W - 0 */
    TSI_PEN_PEN0              = 1 << 0,    /* R/W - 0 */
} tsiPen_t;

typedef enum {
    TSI_STATUS_ERROF15        = 1 << 31,   /* R/W - 0 */
    TSI_STATUS_ERROF14        = 1 << 30,   /* R/W - 0 */
    TSI_STATUS_ERROF13        = 1 << 29,   /* R/W - 0 */
    TSI_STATUS_ERROF12        = 1 << 28,   /* R/W - 0 */
    TSI_STATUS_ERROF11        = 1 << 27,   /* R/W - 0 */
    TSI_STATUS_ERROF10        = 1 << 26,   /* R/W - 0 */
    TSI_STATUS_ERROF9         = 1 << 25,   /* R/W - 0 */
    TSI_STATUS_ERROF8         = 1 << 24,   /* R/W - 0 */
    TSI_STATUS_ERROF7         = 1 << 23,   /* R/W - 0 */
    TSI_STATUS_ERROF6         = 1 << 22,   /* R/W - 0 */
    TSI_STATUS_ERROF5         = 1 << 21,   /* R/W - 0 */
    TSI_STATUS_ERROF4         = 1 << 20,   /* R/W - 0 */
    TSI_STATUS_ERROF3         = 1 << 19,   /* R/W - 0 */
    TSI_STATUS_ERROF2         = 1 << 18,   /* R/W - 0 */
    TSI_STATUS_ERROF1         = 1 << 17,   /* R/W - 0 */
    TSI_STATUS_ERROF0         = 1 << 16,   /* R/W - 0 */
    TSI_STATUS_ORNGF15        = 1 << 15,   /* R/W - 0 */
    TSI_STATUS_ORNGF14        = 1 << 14,   /* R/W - 0 */
    TSI_STATUS_ORNGF13        = 1 << 13,   /* R/W - 0 */
    TSI_STATUS_ORNGF12        = 1 << 12,   /* R/W - 0 */
    TSI_STATUS_ORNGF11        = 1 << 11,   /* R/W - 0 */
    TSI_STATUS_ORNGF10        = 1 << 10,   /* R/W - 0 */
    TSI_STATUS_ORNGF9         = 1 << 9,    /* R/W - 0 */
    TSI_STATUS_ORNGF8         = 1 << 8,    /* R/W - 0 */
    TSI_STATUS_ORNGF7         = 1 << 7,    /* R/W - 0 */
    TSI_STATUS_ORNGF6         = 1 << 6,    /* R/W - 0 */
    TSI_STATUS_ORNGF5         = 1 << 5,    /* R/W - 0 */
    TSI_STATUS_ORNGF4         = 1 << 4,    /* R/W - 0 */
    TSI_STATUS_ORNGF3         = 1 << 3,    /* R/W - 0 */
    TSI_STATUS_ORNGF2         = 1 << 2,    /* R/W - 0 */
    TSI_STATUS_ORNGF1         = 1 << 1,    /* R/W - 0 */
    TSI_STATUS_ORNGF0         = 1 << 0,    /* R/W - 0 */
} tsiStatus_t;

typedef enum {
    TSI_CNTR_ODD_MASK         = 0xFFFF << 16, /* R/W - 0 */
    TSI_CNTR_ODD_SHIFT        = 16,
    TSI_CNTR_EVEN_MASK        = 0xFFFF << 0,  /* R/W - 0 */
    TSI_CNTR_EVEN_SHIFT       = 0,
} tsiCntr_t;

typedef enum {
    TSI_THRESHOLD_LTHH_MASK   = 0xFFFF << 16, /* R/W - 0 */
    TSI_THRESHOLD_LTHH_SHIFT  = 16,
    TSI_THRESHOLD_HTHH_MASK   = 0xFFFF << 0,  /* R/W - 0 */
    TSI_THRESHOLD_HTHH_SHIFT  = 0,
} tsiThreshold_t;

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

/*******************************************************************************
* CLOCKS
*
* MCG     = Multipurpose Clock Generator
* OSC     = Oscillator
* RTC     = Real Time Clock
*
*******************************************************************************/

typedef enum {
    MCGIRCLK,
    MCGFFCLK,
    SYSTEM,                              /* Clocks the ARM Cortex M4 core */
    CORE = SYSTEM,
    BUS,
    FLEXBUS,
    FLASH,
    MCGPLLCLK,
    MCGFLLCLK,
    OSCERCLK,
    ERCLK32K,
    LPO,
} clockSource_t;

                                        /* Multipurpose Clock Generator (MCG) */
typedef struct {
    uint8_t c1;                 /* Control 1 Register                     0x0 */
    uint8_t c2;                 /* Control 2 Register                     0x1 */
    uint8_t c3;                 /* Control 3 Register                     0x2 */
    uint8_t c4;                 /* Control 4 Register                     0x3 */
    uint8_t c5;                 /* Control 5 Register                     0x4 */
    uint8_t c6;                 /* Control 6 Register                     0x5 */
    uint8_t s;                  /* Status Register                        0x6 */
    uint8_t const _unused1;     /*                                        0x7 */
    uint8_t atc;                /* Auto Trim Control Register             0x8 */
    uint8_t const _unused2;     /*                                        0x9 */
    uint8_t atcvh;              /* Auto Trim Compare Value High Register  0xA */
    uint8_t atcvl;              /* Auto Trim Compare Value Low Register   0xB */
} mcg_t;

#define MCG_BASE_ADDR 0x40064000

typedef enum {
    /* MCG Control 1 Register */
    MCG_C1_CLKS_MASK      = 0x3 << 6,
    MCG_C1_FRDIV_MASK     = 0x7 << 3,
    MCG_C1_IREFS          = BIT_2,
    MCG_C1_IRCLKEN        = BIT_1,
    MCG_C1_IREFSTEN       = BIT_0,
    /* MCG Control 2 Register */
    MCG_C2_RANGE_MASK     = 0x3 << 4,
    MCG_C2_HGO            = BIT_3,
    MCG_C2_EREFS          = BIT_2,
    MCG_C2_LP             = BIT_1,
    MCG_C2_IRCS           = BIT_0,
    /* MCG Control 3 Register */
    MCG_C3_SCTRIM_MASK    = 0xFF,
    /* MCG Control 4 Register */
    MCG_C4_DMX32          = BIT_7,
    MCG_C4_DRST_DRS_MASK  = 0x3 << 5,
    MCG_C4_FCTRIM_MASK    = 0xF << 1,
    MCG_C4_SCFTRIM        = BIT_0,
    /* MCG Control 5 Register */
    MCG_C5_PLLCLKEN       = BIT_6,
    MCG_C5_PLLSTEN        = BIT_5,
    MCG_C5_PRDIV_MASK     = 0x1F,
    /* MCG Control 6 Register */
    MCG_C6_LOLIE          = BIT_7,
    MCG_C6_PLLS           = BIT_6,
    MCG_C6_CME            = BIT_5,
    MCG_C6_VDIV_MASK      = 0x1F,
} mcgControlReg_t;

/* MCG Status Register */
typedef enum {
    MCG_S_LOLS            = BIT_7,
    MCG_S_LOCK            = BIT_6,
    MCG_S_PLLST           = BIT_5,
    MCG_S_IREFST          = BIT_4,
    MCG_S_CLKST_MASK      = 0x3 << 2,
    MCG_S_OSCINIT         = BIT_1,
    MCG_S_IRCST           = BIT_0,
} mcgStatusReg_t;

typedef enum {
    /* MCG Auto Trim Control register */
    MCG_ATC_ATME          = BIT_7,
    MCG_ATC_ATMS          = BIT_6,
    MCG_ATC_ATMF          = BIT_5,
    /* MCG Auto Trim Compare Value High Register */
    MCG_ATCVH             = 0xFF,
    /* MCG Auto Trim Compare Value Low Register */
    MCG_ATCVL             = 0xFF,
} mcgAutoTrimReg_t;

                                                          /* Oscillator (OSC) */
typedef struct {
    uint8_t cr;                  /* Control Register                      0x0 */
} osc_t;

#define OSC_BASE_ADDR 0x40065000

/* OSC Control register */
typedef enum {
    OSC_CR_ERCLKEN        = BIT_7,
    OSC_CR_EREFSTEN       = BIT_5,
    OSC_CR_SC2P           = BIT_3,
    OSC_CR_SC4P           = BIT_2,
    OSC_CR_SC8P           = BIT_1,
    OSC_CR_SC16P          = BIT_0,
} oscControlReg_t;
                                                     /* Real Time Clock (RTC) */
typedef struct {
    uint32_t tsr;                /* Time Seconds Register               0x000 */
    uint32_t tpr;                /* Time Prescaler Register             0x004 */
    uint32_t tar;                /* Time Alarm Register                 0x008 */
    uint32_t tcr;                /* Time Compensation Register          0x00C */
    uint32_t cr;                 /* Control Register                    0x010 */
    uint32_t sr;                 /* Status Register                     0x014 */
    uint32_t lr;                 /* Lock Register                       0x018 */
    uint32_t ier;                /* Interrupt Enable Register           0x01C */
    uint32_t const _unused[63];  /*                                     0x020 */
    uint32_t war;                /* Write Access Register               0x800 */
    uint32_t rar;                /* Read Access Register                0x804 */
} rtc_t;

#define RTC_BASE_ADDR 0x4003D000

typedef enum {
    /* RTC Time Seconds Register */
    RTC_TSR_MASK          = 0xFFFFFFFF,
    /* RTC Time Prescaler Register */
    RTC_TPR_MASK          = 0xFFFF,
    /* RTC Time Alarm Register */
    RTC_TAR_MASK          = 0xFFFFFFFF,
    /* RTC Time Compensation Register */
    RTC_TCR_CIC_MASK      = 0xFF << 24,
    RTC_TCR_TCV_MASK      = 0xFF << 16,
    RTC_TCR_CIR_MASK      = 0xFF << 8,
    RTC_TCR_TCR_MASK      = 0xFF,
} rtcTimeReg_t;

/* RTC Control Register */
typedef enum {
    RTC_CR_SC2P           = BIT_13,
    RTC_CR_SC4P           = BIT_12,
    RTC_CR_SC8P           = BIT_11,
    RTC_CR_SC16P          = BIT_10,
    RTC_CR_CLKO           = BIT_9,
    RTC_CR_OSCE           = BIT_8,
    RTC_CR_UM             = BIT_3,
    RTC_CR_SUP            = BIT_2,
    RTC_CR_WEP            = BIT_1,
    RTC_CR_SWR            = BIT_0,
} rtcControlReg_t;

/* RTC Status Register */
typedef enum {
    RTC_SR_TCE            = BIT_4,
    RTC_SR_TAF            = BIT_2,
    RTC_SR_TOF            = BIT_1,
    RTC_SR_TIF            = BIT_0,
} rtcStatusReg_t;

/* RTC Lock Register */
typedef enum {
    RTC_LR_LRL            = BIT_6,
    RTC_LR_SRL            = BIT_5,
    RTC_LR_CRL            = BIT_4,
    RTC_LR_TCL            = BIT_3,
} rtcLockReg_t;

/* RTC Interrupt Enable Register */
typedef enum {
    RTC_IER_TAIE          = BIT_2,
    RTC_IER_TOIE          = BIT_1,
    RTC_IER_TIIE          = BIT_0,
} rtcIeReg_t;

typedef enum {
    /* RTC Write Access Register */
    RTC_WAR_IERW          = BIT_7,
    RTC_WAR_LRW           = BIT_6,
    RTC_WAR_SRW           = BIT_5,
    RTC_WAR_CRW           = BIT_4,
    RTC_WAR_TCRW          = BIT_3,
    RTC_WAR_TARW          = BIT_2,
    RTC_WAR_TPRW          = BIT_1,
    RTC_WAR_TSRW          = BIT_0,
    /* RTC Read Access Register */
    RTC_RAR_IERR          = BIT_7,
    RTC_RAR_LRR           = BIT_6,
    RTC_RAR_SRR           = BIT_5,
    RTC_RAR_CRR           = BIT_4,
    RTC_RAR_TCRR          = BIT_3,
    RTC_RAR_TARR          = BIT_2,
    RTC_RAR_TPRR          = BIT_1,
    RTC_RAR_TSRR          = BIT_0,
} rtcAccessReg_t;


#endif
