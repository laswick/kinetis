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
* Memory Map
*******************************************************************************/
#define MEMMAP_READ_ONLY_ADDR       0x00000000
#define MEMMAP_FLEXNVM_ADDR         0x12000000
#define MEMMAP_FLEXRAM_ADDR         0x14000000
#define MEMMAP_PROG_ACCEL_RAM_ADDR  MEMMAP_FLEXRAM_ADDR
#define MEMMAP_SRAM_L_ADDR          0x18000000
#define MEMMAP_SRAM_U_ADDR          0x20000000
#define MEMMAP_BB_SRAM_U_ADDR       0x22000000  /* Bit Band Alias Region */
#define MEMMAP_PBRIDGE0_BASE_ADDR   0x40000000
#define MEMMAP_PBRIDGE1_BASE_ADDR   0x40080000
#define MEMMAP_GPIO_BASE_ADDR       0x400FF000
#define MEMMAP_BB_GPIO_PBRIDGE_ADDR 0x42000000  /* Bit Band Alias Region */
#define MEMMAP_PRIV_PERIPH_ADD      0xE0000000


/*******************************************************************************
* Bit Bands
*******************************************************************************/

#define BB1_ADDR (MEMMAP_BB_SRAM_U_ADDR)
#define BB1_SRC_ADDR (MEMMAP_SRAM_U_ADDR)
#define BB2_ADDR (MEMMAP_BB_GPIO_PBRIDGE_ADDR)
#define BB2_SRC_ADDR (MEMMAP_PBRIDGE0_BASE_ADDR)

/* Bit To Bit-Band Address Macro */
#define BIT_2_BB1_ADDR(regAddr, bitNum) ((((regAddr)-BB1_SRC_ADDR)*32U) + \
                                                       BB1_ADDR + ((bitNum)*4U))
#define BIT_2_BB2_ADDR(regAddr, bitNum) ((((regAddr)-BB2_SRC_ADDR)*32U) + \
                                                       BB2_ADDR + ((bitNum)*4U))


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


/********** SIM_SCGC1 **********/
#define SIM_SCGC1_ADDR  0x40048028
#define SIM_SCGC1_PTR     (volatile uint32_t *) SIM_SCGC1_ADDR
#define SIM_SCGC1       (*(volatile uint32_t *) SIM_SCGC1_ADDR)
#define SIM_SCGC1_UART5_ENABLE  BIT_11
#define SIM_SCGC1_UART4_ENABLE  BIT_10

/********** SIM_SCGC2 **********/
#define SIM_SCGC2_ADDR  0x4004802C
#define SIM_SCGC2_PTR     (volatile uint32_t *) SIM_SCGC2_ADDR
#define SIM_SCGC2       (*(volatile uint32_t *) SIM_SCGC2_ADDR)
#define SIM_SCGC2_DAC0_ENABLE  BIT_12

/***** SIM System Clock Gate Control Register 3 *****/
                                           /* Register Address & Bit Numbers */
#define SIM_SCGC3_ADDR           0x40048030
#define SIM_SCGC3_PTR            ((volatile uint32_t *) SIM_SCGC3_ADDR)
#define SIM_SCGC3                (*(SIM_SCGC3_PTR))
#define SIM_SCGC3_BB_ADDR        (MEMMAP_BB_GPIO_PBRIDGE_ADDR + (0x48030 * 32U))
                                                              /* Bit Numbers */
#define SIM_SCGC3_ADC1_BIT       23
#define SIM_SCGC3_SPI2_BIT       12
                                                                    /* Masks */
#define SIM_SCGC3_ADC1_MSK       (1<<SIM_SCGC3_ADC1_BIT)
#define SIM_SCGC3_SPI2_MSK       (1<<SIM_SCGC3_SPI2_BIT)
                                                     /* Bit-Banded Addresses */
#define SIM_SCGC3_ADC1_BB_ADDR   (SIM_SCGC3_BB_ADDR +(SIM_SCGC3_ADC1_BIT  * 4))
#define SIM_SCGC3_SPI2_BB_ADDR   (SIM_SCGC3_BB_ADDR +(SIM_SCGC3_SPI2_BIT * 4))
                                                      /* Bit-Banded Pointers */
#define SIM_SCGC3_ADC1_BB_PTR    ((volatile uint32_t *) SIM_SCGC3_ADC1_BB_ADDR)
#define SIM_SCGC3_SPI2_BB_PTR    ((volatile uint32_t *) SIM_SCGC3_SPI2_BB_ADDR)
                                         /* Bit-Banded Dereferenced Pointers */
#define SIM_SCGC3_ADC1_BB        (*(SIM_SCGC3_ADC1_BB_PTR))
#define SIM_SCGC3_SPI2_BB        (*(SIM_SCGC3_SPI2_BB_PTR))

#define SIM_SCGC3_SPI2_ENABLE  BIT_12
#define SIM_SCGC3_ADC1_ENABLE  BIT_27
#define SIM_SCGC3_SDHC_ENABLE  BIT_17

/********** SIM_SCGC4 **********/
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

/********** SIM_SCGC5 **********/
#define SIM_SCGC5_ADDR  0x40048038
#define SIM_SCGC5_PTR     (volatile uint32_t *) SIM_SCGC5_ADDR
#define SIM_SCGC5       (*(volatile uint32_t *) SIM_SCGC5_ADDR)
#define SIM_SCGC5_TSI_ENABLE   BIT_5
#define SIM_SCGC5_PORTA_ENABLE BIT_9
#define SIM_SCGC5_PORTB_ENABLE BIT_10
#define SIM_SCGC5_PORTC_ENABLE BIT_11
#define SIM_SCGC5_PORTD_ENABLE BIT_12
#define SIM_SCGC5_PORTE_ENABLE BIT_13

/***** SIM System Clock Gate Control Register 6 *****/
                                           /* Register Address & Bit Numbers */
#define SIM_SCGC6_ADDR          0x4004803C
#define SIM_SCGC6_PTR           ((volatile uint32_t *) SIM_SCGC6_ADDR)
#define SIM_SCGC6               (*(SIM_SCGC6_PTR))
#define SIM_SCGC6_BB_ADDR       (MEMMAP_BB_GPIO_PBRIDGE_ADDR + (0x4803C * 32U))
                                                              /* Bit Numbers */
#define SIM_SCGC6_PIT_BIT       23
#define SIM_SCGC6_CRC_BIT       18
#define SIM_SCGC6_SPI1_BIT      13
#define SIM_SCGC6_SPI0_BIT      12
#define SIM_SCGC6_PIT_ENABLE   BIT_23
#define SIM_SCGC6_ADC0_ENABLE  BIT_27
#define SIM_SCGC6_FTFL_ENABLE  BIT_0
#define SIM_SCGC6_DMAMUX_BIT    1
                                                                    /* Masks */
#define SIM_SCGC6_PIT_MSK       (1<<SIM_SCGC6_PIT_BIT)
#define SIM_SCGC6_CRC_MSK       (1<<SIM_SCGC6_CRC_BIT)
#define SIM_SCGC6_SPI1_MSK      (1<<SIM_SCGC6_SPI1_BIT)
#define SIM_SCGC6_SPI0_MSK      (1<<SIM_SCGC6_SPI0_BIT)
#define SIM_SCGC6_DMAMUX_MSK    (1<<SIM_SCGC6_DMAMUX_BIT)
                                                     /* Bit-Banded Addresses */
#define SIM_SCGC6_CRC_BB_ADDR    (SIM_SCGC6_BB_ADDR +(SIM_SCGC6_CRC_BIT  * 4))
#define SIM_SCGC6_SPI1_BB_ADDR   (SIM_SCGC6_BB_ADDR +(SIM_SCGC6_SPI1_BIT * 4))
#define SIM_SCGC6_SPI0_BB_ADDR   (SIM_SCGC6_BB_ADDR +(SIM_SCGC6_SPI0_BIT * 4))
#define SIM_SCGC6_DMAMUX_BB_ADDR (SIM_SCGC6_BB_ADDR +(SIM_SCGC6_DMAMUX_BIT * 4))
                                                      /* Bit-Banded Pointers */
#define SIM_SCGC6_CRC_BB_PTR    ((volatile uint32_t *) SIM_SCGC6_CRC_BB_ADDR)
#define SIM_SCGC6_SPI1_BB_PTR   ((volatile uint32_t *) SIM_SCGC6_SPI1_BB_ADDR)
#define SIM_SCGC6_SPI0_BB_PTR   ((volatile uint32_t *) SIM_SCGC6_SPI0_BB_ADDR)
#define SIM_SCGC6_DMAMUX_BB_PTR ((volatile uint32_t *) SIM_SCGC6_DMAMUX_BB_ADDR)
                                         /* Bit-Banded Dereferenced Pointers */
#define SIM_SCGC6_CRC_BB        (*(SIM_SCGC6_CRC_BB_PTR))
#define SIM_SCGC6_SPI1_BB       (*(SIM_SCGC6_SPI1_BB_PTR))
#define SIM_SCGC6_SPI0_BB       (*(SIM_SCGC6_SPI0_BB_PTR))
#define SIM_SCGC6_DMAMUX_BB     (*(SIM_SCGC6_DMAMUX_BB_PTR))


/***** SIM System Clock Gate Control Register 7 *****/
                                           /* Register Address & Bit Numbers */

#define SIM_SCGC7_ADDR          0x40048040
#define SIM_SCGC7_PTR           ((volatile uint32_t *) SIM_SCGC7_ADDR)
#define SIM_SCGC7               (*(SIM_SCGC7_PTR))
#define SIM_SCGC7_BB_ADDR       (MEMMAP_BB_GPIO_PBRIDGE_ADDR + (0x48040 * 32U))
                                                              /* Bit Numbers */
#define SIM_SCGC7_DMA_BIT       1
#define SIM_SCGC7_FLEXBUS_ENABLE BIT_0
                                                                    /* Masks */
#define SIM_SCGC7_DMA_MSK       (1<<SIM_SCGC7_DMA_BIT)
                                                     /* Bit-Banded Addresses */
#define SIM_SCGC7_DMA_BB_ADDR    (SIM_SCGC7_BB_ADDR +(SIM_SCGC7_DMA_BIT  * 4))
                                                      /* Bit-Banded Pointers */
#define SIM_SCGC7_DMA_BB_PTR    ((volatile uint32_t *) SIM_SCGC7_DMA_BB_ADDR)
                                         /* Bit-Banded Dereferenced Pointers */



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
    ADC_SC2_ADACT_BIT      = 1 << 7,
    ADC_SC2_ADTRG_BIT      = 1 << 6, /* Set for hardware trigger */
    ADC_SC2_ACFE_BIT       = 1 << 5, /* Compare function enable */
    ADC_SC2_ACFGT_BIT      = 1 << 4, /* Compare function > enable */
    ADC_SC2_ADREN_BIT      = 1 << 3, /* Compare function range enable */
    ADC_SC2_DMAEN_BIT      = 1 << 2, /* DMA enable */

    ADC_SC2_REFSEL_ALT_BIT = 1 << 0, /* V_ALTH, V_ALTL */
};

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
* WATCH DOG
*******************************************************************************/
typedef struct {
    uint16_t stCtrlFlags;
    uint16_t prescaler;
    uint32_t timeout;
    uint32_t window;
} watchDogConfig_t;

/* Watchdog Status and Control Register High enum */
enum {
   WDOG_EN          = BIT_0,
   WDOG_CLKSRC      = BIT_1,
   WDOG_IRQRSTEN    = BIT_2,            /* TODO */
   WDOG_WINEN       = BIT_3,
   WDOG_ALLOWUPDATE = BIT_4,
   WDOG_DBGEN       = BIT_5,
   WDOG_STOPEN      = BIT_6,
   WDOG_WAITEN      = BIT_7,
   WDOG_STNDBYEN    = BIT_8,

   WDOG_TEST        = BIT_10,           /* TODO */
   WDOG_TESTSEL     = BIT_11,           /* TODO */
   WDOG_BYTES_SEL   = BIT_12 | BIT_13,  /* TODO */
   WDOG_DISTEST     = BIT_14,           /* TODO */
};

/*******************************************************************************
* VREF
*******************************************************************************/

#define VREF_BASE_ADDR 0x40074000
#define VREF_REG_PTR   (volatile vref_t *) VREF_BASE_ADDR

typedef struct {
    uint8_t trm; /* Voltage trim register VREF_TRM , R/W, factory trim */
    uint8_t sc;  /* Status and control register,     R/W, 0x00000000 */
} vref_t; /* Memmapped registers */

#define VREF_TRM_TRIM_MASK 0x3F /* Factory trimmed to 1.2V.  ~ +/- 0.5mV/bit */

enum {
    VREF_SC_VREFEN     = BIT_7,
    VREF_SC_REFEN      = BIT_6,

    VREF_SC_VREFST     = BIT_2,

    VREF_SC_MODE_TIGHT = BIT_1,
};
#define VREF_SC_PGA_SUPPORT ((uint8_t) (VREF_SC_VREFEN | VREF_SC_REFEN \
                                                       | VREF_SC_MODE_TIGHT))

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

/***** Common to All SPI *****/
                                                              /* Bit Numbers */
/* MCR */
#define SPI_MCR_MSTR_BIT         31U /* R/W - 0*/
#define SPI_MCR_CONT_SCKE_BIT    30U /* R/W - 0*/
#define SPI_MCR_DCONF1_BIT       29U /* R/W - 0*/
#define SPI_MCR_DCONF0_BIT       28U /* R/W - 0*/
#define SPI_MCR_FRZ_BIT          27U /* R/W - 0*/
#define SPI_MCR_MTFE_BIT         26U /* R/W - 0*/
#define SPI_MCR_PCSSE_BIT        25U /* R/W - 0*/
#define SPI_MCR_ROOE_BIT         24U /* R/W - 0*/
#define SPI_MCR_PCSIS5_BIT       21U /* R/W - 0*/
#define SPI_MCR_PCSIS4_BIT       20U /* R/W - 0*/
#define SPI_MCR_PCSIS3_BIT       19U /* R/W - 0*/
#define SPI_MCR_PCSIS2_BIT       18U /* R/W - 0*/
#define SPI_MCR_PCSIS1_BIT       17U /* R/W - 0*/
#define SPI_MCR_PCSIS0_BIT       16U /* R/W - 0*/
#define SPI_MCR_DOZE_BIT         15U /* R/W - 0*/
#define SPI_MCR_MDIS_BIT         14U /* R/W - 1*/
#define SPI_MCR_DIS_TXF_BIT      13U /* R/W - 0*/
#define SPI_MCR_DIS_RXF_BIT      12U /* R/W - 0*/
#define SPI_MCR_CLR_TXF_BIT      11U /*   W - 0*/
#define SPI_MCR_CLR_RXF_BIT      10U /*   W - 0*/
#define SPI_MCR_SMPL_PT1_BIT     9U  /* R/W - 0*/
#define SPI_MCR_SMPL_PT0_BIT     8U  /* R/W - 0*/
#define SPI_MCR_HALT_BIT         0U  /* R/W - 1*/
/* CTAR */
#define SPI_CTAR_DBR_BIT         31U /* R/W - 0*/
#define SPI_CTAR_FMSZ3_BIT       30U /* R/W - 1*/
#define SPI_CTAR_FMSZ2_BIT       29U /* R/W - 1*/
#define SPI_CTAR_FMSZ1_BIT       28U /* R/W - 1*/
#define SPI_CTAR_FMSZ0_BIT       27U /* R/W - 1*/
#define SPI_CTAR_CPOL_BIT        26U /* R/W - 0*/
#define SPI_CTAR_CPHA_BIT        25U /* R/W - 0*/
#define SPI_CTAR_LSBFE_BIT       24U /* R/W - 0*/
#define SPI_CTAR_PCSSCK1_BIT     23U /* R/W - 0*/
#define SPI_CTAR_PCSSCK0_BIT     22U /* R/W - 0*/
#define SPI_CTAR_PASC1_BIT       21U /* R/W - 0*/
#define SPI_CTAR_PASC0_BIT       20U /* R/W - 0*/
#define SPI_CTAR_PDT1_BIT        19U /* R/W - 0*/
#define SPI_CTAR_PDT0_BIT        18U /* R/W - 0*/
#define SPI_CTAR_PBR1_BIT        17U /* R/W - 0*/
#define SPI_CTAR_PBR0_BIT        16U /* R/W - 0*/
#define SPI_CTAR_CSSCLK3_BIT     15U /* R/W - 0*/
#define SPI_CTAR_CSSCLK2_BIT     14U /* R/W - 0*/
#define SPI_CTAR_CSSCLK1_BIT     13U /* R/W - 0*/
#define SPI_CTAR_CSSCLK0_BIT     12U /* R/W - 0*/
#define SPI_CTAR_ASC3_BIT        11U /* R/W - 0*/
#define SPI_CTAR_ASC2_BIT        10U /* R/W - 0*/
#define SPI_CTAR_ASC1_BIT        9U  /* R/W - 0*/
#define SPI_CTAR_ASC0_BIT        8U  /* R/W - 0*/
#define SPI_CTAR_DT3_BIT         7U  /* R/W - 0*/
#define SPI_CTAR_DT2_BIT         6U  /* R/W - 0*/
#define SPI_CTAR_DT1_BIT         5U  /* R/W - 0*/
#define SPI_CTAR_DT0_BIT         4U  /* R/W - 0*/
#define SPI_CTAR_BR3_BIT         3U  /* R/W - 0*/
#define SPI_CTAR_BR2_BIT         2U  /* R/W - 0*/
#define SPI_CTAR_BR1_BIT         1U  /* R/W - 0*/
#define SPI_CTAR_BR0_BIT         0U  /* R/W - 0*/
/* PUSHR */
#define SPI_PUSHR_CONT_BIT       31U /* R/W - 0*/
#define SPI_PUSHR_CTAS2_BIT      30U /* R/W - 0*/
#define SPI_PUSHR_CTAS1_BIT      29U /* R/W - 0*/
#define SPI_PUSHR_CTAS0_BIT      28U /* R/W - 0*/
#define SPI_PUSHR_EOQ_BIT        27U /* R/W - 0*/
#define SPI_PUSHR_CTCNT_BIT      26U /* R/W - 0*/
#define SPI_PUSHR_PCS5_BIT       21U /* R/W - 0*/
#define SPI_PUSHR_PCS4_BIT       20U /* R/W - 0*/
#define SPI_PUSHR_PCS3_BIT       19U /* R/W - 0*/
#define SPI_PUSHR_PCS2_BIT       18U /* R/W - 0*/
#define SPI_PUSHR_PCS1_BIT       17U /* R/W - 0*/
#define SPI_PUSHR_PCS0_BIT       16U /* R/W - 0*/
/* RSER */
#define SPI_RSER_TCF_RE_BIT      31U /* R/W - 0*/
#define SPI_RSER_EOQF_RE_BIT     28U /* R/W - 0*/
#define SPI_RSER_TFUF_RE_BIT     27U /* R/W - 0*/
#define SPI_RSER_TFFF_RE_BIT     25U /* R/W - 0*/
#define SPI_RSER_TFFF_DIRS_BIT   24U /* R/W - 0*/
#define SPI_RSER_RFOF_RE_BIT     19U /* R/W - 0*/
#define SPI_RSER_RFDF_RE_BIT     17U /* R/W - 0*/
#define SPI_RSER_RFDF_DIRS_BIT   16U /* R/W - 0*/
/* SR - Status Register */
#define SPI_SR_TCF_BIT           31U /* R/W - 0*/
#define SPI_SR_TXRXS_BIT         30U /* R/W - 0*/
#define SPI_SR_EOQF_BIT          28U /* R/W - 0*/
#define SPI_SR_TFUF_BIT          27U /* R/W - 0*/
#define SPI_SR_TFFF_BIT          25U /* R/W - 0*/
#define SPI_SR_RFOF_BIT          19U /* R/W - 0*/
#define SPI_SR_RFDF_BIT          17U /* R/W - 0*/
#define SPI_SR_TXCTR3_BIT        15U /* R/W - 0*/
#define SPI_SR_TXCTR2_BIT        14U /* R/W - 0*/
#define SPI_SR_TXCTR1_BIT        13U /* R/W - 0*/
#define SPI_SR_TXCTR0_BIT        12U /* R/W - 0*/
#define SPI_SR_TXNXTPTR3_BIT     11U /* R/W - 0*/
#define SPI_SR_TXNXTPTR2_BIT     10U /* R/W - 0*/
#define SPI_SR_TXNXTPTR1_BIT     9U  /* R/W - 0*/
#define SPI_SR_TXNXTPTR0_BIT     8U  /* R/W - 0*/
#define SPI_SR_RXCTR3_BIT        7U  /* R/W - 0*/
#define SPI_SR_RXCTR2_BIT        6U  /* R/W - 0*/
#define SPI_SR_RXCTR1_BIT        5U  /* R/W - 0*/
#define SPI_SR_RXCTR0_BIT        4U  /* R/W - 0*/
#define SPI_SR_POPNXTPTR3_BIT    3U  /* R/W - 0*/
#define SPI_SR_POPNXTPTR2_BIT    2U  /* R/W - 0*/
#define SPI_SR_POPNXTPTR1_BIT    1U  /* R/W - 0*/
#define SPI_SR_POPNXTPTR0_BIT    0U  /* R/W - 0*/
                                                                    /* Masks */
#define SPI_MCR_MSTR_MSK         (1U<<SPI_MCR_MSTR_BIT)
#define SPI_MCR_CONT_SCKE_MSK    (1U<<SPI_MCR_CONT_SCKE_BIT)
#define SPI_MCR_DCONF1_MSK       (1U<<SPI_MCR_DCONF1_BIT)
#define SPI_MCR_DCONF0_MSK       (1U<<SPI_MCR_DCONF0_BIT)
#define SPI_MCR_DCONF_MSK        (SPI_MCR_DCONF0_MSK | SPI_MCR_DCONF1_MSK)
#define SPI_MCR_FRZ_MSK          (1U<<SPI_MCR_FRZ_BIT)
#define SPI_MCR_MTFE_MSK         (1U<<SPI_MCR_MTFE_BIT)
#define SPI_MCR_PCSSE_MSK        (1U<<SPI_MCR_PCSSE_BIT)
#define SPI_MCR_ROOE_MSK         (1U<<SPI_MCR_ROOE_BIT)
#define SPI_MCR_PCSIS5_MSK       (1U<<SPI_MCR_PCSIS5_BIT)
#define SPI_MCR_PCSIS4_MSK       (1U<<SPI_MCR_PCSIS4_BIT)
#define SPI_MCR_PCSIS3_MSK       (1U<<SPI_MCR_PCSIS3_BIT)
#define SPI_MCR_PCSIS2_MSK       (1U<<SPI_MCR_PCSIS2_BIT)
#define SPI_MCR_PCSIS1_MSK       (1U<<SPI_MCR_PCSIS1_BIT)
#define SPI_MCR_PCSIS0_MSK       (1U<<SPI_MCR_PCSIS0_BIT)
#define SPI_MCR_PCSIS_MSK        (SPI_MCR_PCSIS0_MSK | SPI_MCR_PCSIS1_MSK |\
                                  SPI_MCR_PCSIS2_MSK | SPI_MCR_PCSIS3_MSK |\
                                  SPI_MCR_PCSIS4_MSK | SPI_MCR_PCSIS5_MSK  )
#define SPI_MCR_DOZE_MSK         (1U<<SPI_MCR_DOZE_BIT)
#define SPI_MCR_MDIS_MSK         (1U<<SPI_MCR_MDIS_BIT)
#define SPI_MCR_DIS_TXF_MSK      (1U<<SPI_MCR_DIS_TXF_BIT)
#define SPI_MCR_DIS_RXF_MSK      (1U<<SPI_MCR_DIS_RXF_BIT)
#define SPI_MCR_CLR_TXF_MSK      (1U<<SPI_MCR_CLR_TXF_BIT)
#define SPI_MCR_CLR_RXF_MSK      (1U<<SPI_MCR_CLR_RXF_BIT)
#define SPI_MCR_SMPL_PT1_MSK     (1U<<SPI_MCR_SMPL_PT1_BIT)
#define SPI_MCR_SMPL_PT0_MSK     (1U<<SPI_MCR_SMPL_PT0_BIT)
#define SPI_MCR_SMPL_PT_MSK      (SPI_MCR_SMPL_PT0_MSK | SPI_MCR_SMPL_PT1_MSK)
#define SPI_MCR_HALT_MSK         (1U<<SPI_MCR_HALT_BIT)

#define SPI_CTAR_FMSZ3_MSK       (1U<<SPI_CTAR_FMSZ3_BIT)
#define SPI_CTAR_FMSZ2_MSK       (1U<<SPI_CTAR_FMSZ2_BIT)
#define SPI_CTAR_FMSZ1_MSK       (1U<<SPI_CTAR_FMSZ1_BIT)
#define SPI_CTAR_FMSZ0_MSK       (1U<<SPI_CTAR_FMSZ0_BIT)
#define SPI_CTAR_FMSZ_MSK        (SPI_CTAR_FMSZ0_MSK | SPI_CTAR_FMSZ1_MSK |\
                                  SPI_CTAR_FMSZ2_MSK | SPI_CTAR_FMSZ3_MSK  )
#define SPI_CTAR_CPOL_MSK        (1U<<SPI_CTAR_CPOL_BIT)
#define SPI_CTAR_CPHA_MSK        (1U<<SPI_CTAR_CPHA_BIT)
#define SPI_CTAR_LSBFE_MSK       (1U<<SPI_CTAR_LSBFE_BIT)
#define SPI_CTAR_PCSSCK1_MSK     (1U<<SPI_CTAR_PCSSCK1_BIT)
#define SPI_CTAR_PCSSCK0_MSK     (1U<<SPI_CTAR_PCSSCK0_BIT)
#define SPI_CTAR_PCSSCK_MSK      (SPI_CTAR_PCSSCK0_MSK | SPI_CTAR_PCSSCK1_MSK)
#define SPI_CTAR_PASC1_MSK       (1U<<SPI_CTAR_PASC1_BIT)
#define SPI_CTAR_PASC0_MSK       (1U<<SPI_CTAR_PASC0_BIT)
#define SPI_CTAR_PASC_MSK        (SPI_CTAR_PASC0_MSK | SPI_CTAR_PASC1_MSK)
#define SPI_CTAR_PDT1_MSK        (1U<<SPI_CTAR_PDT1_BIT)
#define SPI_CTAR_PDT0_MSK        (1U<<SPI_CTAR_PDT0_BIT)
#define SPI_CTAR_PDT_MSK         (SPI_CTAR_PDT0_MSK | SPI_CTAR_PDT1_MSK)
#define SPI_CTAR_PBR1_MSK        (1U<<SPI_CTAR_PBR1_BIT)
#define SPI_CTAR_PBR0_MSK        (1U<<SPI_CTAR_PBR0_BIT)
#define SPI_CTAR_PBR_MSK         (SPI_CTAR_PBR0_MSK | SPI_CTAR_PBR1_MSK)
#define SPI_CTAR_CSSCLK3_MSK     (1U<<SPI_CTAR_CSSCLK3_BIT)
#define SPI_CTAR_CSSCLK2_MSK     (1U<<SPI_CTAR_CSSCLK2_BIT)
#define SPI_CTAR_CSSCLK1_MSK     (1U<<SPI_CTAR_CSSCLK1_BIT)
#define SPI_CTAR_CSSCLK0_MSK     (1U<<SPI_CTAR_CSSCLK0_BIT)
#define SPI_CTAR_CSSCLK_MSK      (SPI_CTAR_CSSCLK0_MSK | SPI_CTAR_CSSCLK1_MSK |\
                                  SPI_CTAR_CSSCLK2_MSK | SPI_CTAR_CSSCLK3_MSK  )
#define SPI_CTAR_ASC3_MSK        (1U<<SPI_CTAR_ASC3_BIT)
#define SPI_CTAR_ASC2_MSK        (1U<<SPI_CTAR_ASC2_BIT)
#define SPI_CTAR_ASC1_MSK        (1U<<SPI_CTAR_ASC1_BIT)
#define SPI_CTAR_ASC0_MSK        (1U<<SPI_CTAR_ASC0_BIT)
#define SPI_CTAR_ASC_MSK         (SPI_CTAR_ASC0_MSK | SPI_CTAR_ASC1_MSK |\
                                  SPI_CTAR_ASC2_MSK | SPI_CTAR_ASC3_MSK  )
#define SPI_CTAR_DT3_MSK         (1U<<SPI_CTAR_DT3_BIT)
#define SPI_CTAR_DT2_MSK         (1U<<SPI_CTAR_DT2_BIT)
#define SPI_CTAR_DT1_MSK         (1U<<SPI_CTAR_DT1_BIT)
#define SPI_CTAR_DT0_MSK         (1U<<SPI_CTAR_DT0_BIT)
#define SPI_CTAR_DT_MSK          (SPI_CTAR_DT0_MSK | SPI_CTAR_DT1_MSK |\
                                  SPI_CTAR_DT2_MSK | SPI_CTAR_DT3_MSK  )
#define SPI_CTAR_BR3_MSK         (1U<<SPI_CTAR_BR3_BIT)
#define SPI_CTAR_BR2_MSK         (1U<<SPI_CTAR_BR2_BIT)
#define SPI_CTAR_BR1_MSK         (1U<<SPI_CTAR_BR1_BIT)
#define SPI_CTAR_BR0_MSK         (1U<<SPI_CTAR_BR0_BIT)
#define SPI_CTAR_BR_MSK          (SPI_CTAR_BR0_MSK | SPI_CTAR_BR1_MSK |\
                                  SPI_CTAR_BR2_MSK | SPI_CTAR_BR3_MSK  )

#define SPI_PUSHR_CTAS2_MSK      (1U<<SPI_PUSHR_CTAS2_BIT)
#define SPI_PUSHR_CTAS1_MSK      (1U<<SPI_PUSHR_CTAS1_BIT)
#define SPI_PUSHR_CTAS0_MSK      (1U<<SPI_PUSHR_CTAS0_BIT)
#define SPI_PUSHR_CTAS_MSK       (SPI_PUSHR_CTAS0_MSK | SPI_PUSHR_CTAS1_MSK |\
                                                        SPI_PUSHR_CTAS2_MSK  )
#define SPI_PUSHR_EOQ_MSK        (1U<<SPI_PUSHR_EOQ_BIT)
#define SPI_PUSHR_CTCNT_MSK      (1U<<SPI_PUSHR_CTCNT_BIT)
#define SPI_PUSHR_PCS5_MSK       (1U<<SPI_PUSHR_PCS5_BIT)
#define SPI_PUSHR_PCS4_MSK       (1U<<SPI_PUSHR_PCS4_BIT)
#define SPI_PUSHR_PCS3_MSK       (1U<<SPI_PUSHR_PCS3_BIT)
#define SPI_PUSHR_PCS2_MSK       (1U<<SPI_PUSHR_PCS2_BIT)
#define SPI_PUSHR_PCS1_MSK       (1U<<SPI_PUSHR_PCS1_BIT)
#define SPI_PUSHR_PCS0_MSK       (1U<<SPI_PUSHR_PCS0_BIT)
#define SPI_PUSHR_PCS_MSK        (SPI_PUSHR_PCS0_MSK | SPI_PUSHR_PCS1_MSK |\
                                  SPI_PUSHR_PCS2_MSK | SPI_PUSHR_PCS3_MSK |\
                                  SPI_PUSHR_PCS4_MSK | SPI_PUSHR_PCS5_MSK  )

#define SPI_RSER_TCF_RE_MSK      (1U<<SPI_RSER_TCF_RE_BIT)
#define SPI_RSER_EOQF_RE_MSK     (1U<<SPI_RSER_EOQF_RE_BIT)
#define SPI_RSER_TFUF_RE_MSK     (1U<<SPI_RSER_TFUF_RE_BIT)
#define SPI_RSER_TFFF_RE_MSK     (1U<<SPI_RSER_TFFF_RE_BIT)
#define SPI_RSER_TFFF_DIRS_MSK   (1U<<SPI_RSER_TFFF_DIRS_BIT)
#define SPI_RSER_RFOF_RE_MSK     (1U<<SPI_RSER_RFOF_RE_BIT)
#define SPI_RSER_RFDF_RE_MSK     (1U<<SPI_RSER_RFDF_RE_BIT)
#define SPI_RSER_RFDF_DIRS_MSK   (1U<<SPI_RSER_RFDF_DIRS_BIT)

#define SPI_SR_TCF_MSK           (1U<<SPI_SR_TCF_BIT)
#define SPI_SR_TXRXS_MSK         (1U<<SPI_SR_TXRXS_BIT)
#define SPI_SR_EOQF_MSK          (1U<<SPI_SR_EOQF_BIT)
#define SPI_SR_TFUF_MSK          (1U<<SPI_SR_TFUF_BIT)
#define SPI_SR_TFFF_MSK          (1U<<SPI_SR_TFFF_BIT)
#define SPI_SR_RFOF_MSK          (1U<<SPI_SR_RFOF_BIT)
#define SPI_SR_RFDF_MSK          (1U<<SPI_SR_RFDF_BIT)
#define SPI_SR_TXCTR3_MSK        (1U<<SPI_SR_TXCTR3_BIT)
#define SPI_SR_TXCTR2_MSK        (1U<<SPI_SR_TXCTR2_BIT)
#define SPI_SR_TXCTR1_MSK        (1U<<SPI_SR_TXCTR1_BIT)
#define SPI_SR_TXCTR0_MSK        (1U<<SPI_SR_TXCTR0_BIT)
#define SPI_SR_TXCTR_MSK         (SPI_SR_TXCTR0_MSK | SPI_SR_TXCTR1_MSK |\
                                  SPI_SR_TXCTR2_MSK | SPI_SR_TXCTR3_MSK  )
#define SPI_SR_TXNXTPTR3_MSK     (1U<<SPI_SR_TXNXTPTR3_BIT)
#define SPI_SR_TXNXTPTR2_MSK     (1U<<SPI_SR_TXNXTPTR2_BIT)
#define SPI_SR_TXNXTPTR1_MSK     (1U<<SPI_SR_TXNXTPTR1_BIT)
#define SPI_SR_TXNXTPTR0_MSK     (1U<<SPI_SR_TXNXTPTR0_BIT)
#define SPI_SR_TXNXTPTR_MSK      (SPI_SR_TXNXTPTR0_MSK | SPI_SR_TXNXTPTR1_MSK |\
                                  SPI_SR_TXNXTPTR2_MSK | SPI_SR_TXNXTPTR3_MSK  )
#define SPI_SR_RXCTR3_MSK        (1U<<SPI_SR_RXCTR3_BIT)
#define SPI_SR_RXCTR2_MSK        (1U<<SPI_SR_RXCTR2_BIT)
#define SPI_SR_RXCTR1_MSK        (1U<<SPI_SR_RXCTR1_BIT)
#define SPI_SR_RXCTR0_MSK        (1U<<SPI_SR_RXCTR0_BIT)
#define SPI_SR_RXCTR_MSK         (SPI_SR_RXCTR0_MSK | SPI_SR_RXCTR1_MSK |\
                                  SPI_SR_RXCTR2_MSK | SPI_SR_RXCTR3_MSK  )
#define SPI_SR_POPNXTPTR3_MSK    (1U<<SPI_SR_POPNXTPTR3_BIT)
#define SPI_SR_POPNXTPTR2_MSK    (1U<<SPI_SR_POPNXTPTR2_BIT)
#define SPI_SR_POPNXTPTR1_MSK    (1U<<SPI_SR_POPNXTPTR1_BIT)
#define SPI_SR_POPNXTPTR0_MSK    (1U<<SPI_SR_POPNXTPTR0_BIT)
#define SPI_SR_POPNXTPTR_MSK     (SPI_SR_POPNXTPTR0_MSK | SPI_SR_POPNXTPTR1_MSK |\
                                  SPI_SR_POPNXTPTR2_MSK | SPI_SR_POPNXTPTR3_MSK  )

#define SPI0_ADDR           0x4002C000
#define SPI1_ADDR           0x4002D000
#define SPI2_ADDR           0x400AC000

#define SPI_MCR_OFFSET      0x00
#define SPI_CTAR0_OFFSET    0x0C
#define SPI_CTAR1_OFFSET    0x10
#define SPI_SR_OFFSET       0x2C
#define SPI_RSER_OFFSET     0x30
#define SPI_PUSHR_OFFSET    0x34
#define SPI_POPR_OFFSET     0x38

#define SPI_MCR_ADDR(a)      (a + SPI_MCR_OFFSET)
#define SPI_MCR_PTR(a)       ((volatile uint32_t*)SPI_MCR_ADDR((a)))
#define SPI_MCR(a)           (*(SPI_MCR_PTR((a))))
#define SPI_MCR_BB_ADDR(a,b) (BIT_2_BB2_ADDR((SPI_MCR_ADDR((a))),(b)))
#define SPI_MCR_BB_PTR(a,b)  ((volatile uint32_t*)SPI_MCR_BB_ADDR(a,b))
#define SPI_MCR_BB(a,b)      (*(SPI_MCR_BB_PTR(a,b)))

#define SPI_CTAR0_ADDR(a)      (a + SPI_CTAR0_OFFSET)
#define SPI_CTAR0_PTR(a)       ((volatile uint32_t *) SPI_CTAR0_ADDR((a)))
#define SPI_CTAR0(a)           (*(SPI_CTAR0_PTR((a))))
#define SPI_CTAR0_BB_ADDR(a,b) (BIT_2_BB2_ADDR((SPI_CTAR0_ADDR((a))),(b)))
#define SPI_CTAR0_BB_PTR(a,b)  ((volatile uint32_t*)SPI_CTAR0_BB_ADDR(a,b))
#define SPI_CTAR0_BB(a,b)      (*(SPI_CTAR0_BB_PTR(a,b)))

#define SPI_CTAR1_ADDR(a) (a + SPI_CTAR1_OFFSET)
#define SPI_CTAR1_BB(a,b) (BIT_2_BB2_ADDR((SPI_CTAR1_ADDR((a))),(b)))
#define SPI_CTAR1_PTR(a)  ((volatile uint32_t *) SPI_CTAR1_ADDR((a)))
#define SPI_CTAR1(a)      (*(SPI_CTAR1_PTR((a))))

#define SPI_SR_ADDR(a)      (a + SPI_SR_OFFSET)
#define SPI_SR_PTR(a)       ((volatile uint32_t *) SPI_SR_ADDR((a)))
#define SPI_SR(a)           (*(SPI_SR_PTR((a))))
#define SPI_SR_BB_ADDR(a,b) (BIT_2_BB2_ADDR((SPI_SR_ADDR((a))),(b)))
#define SPI_SR_BB_PTR(a,b)  ((volatile uint32_t*)SPI_SR_BB_ADDR(a,b))
#define SPI_SR_BB(a,b)      (*(SPI_SR_BB_PTR(a,b)))

#define SPI_RSER_ADDR(a) (a + SPI_RSER_OFFSET)
#define SPI_RSER_PTR(a)  ((volatile uint32_t *) SPI_RSER_ADDR((a)))
#define SPI_RSER(a)      (*(SPI_RSER_PTR((a))))
#define SPI_RSER_BB_ADDR(a,b) (BIT_2_BB2_ADDR((SPI_RSER_ADDR((a))),(b)))
#define SPI_RSER_BB_PTR(a,b)  ((volatile uint32_t*)SPI_RSER_BB_ADDR(a,b))
#define SPI_RSER_BB(a,b)      (*(SPI_RSER_BB_PTR(a,b)))

#define SPI_PUSHR_ADDR(a) (a + SPI_PUSHR_OFFSET)
#define SPI_PUSHR_BB(a,b) (BIT_2_BB2_ADDR((SPI_PUSHR_ADDR((a))),(b)))
#define SPI_PUSHR_PTR(a)  ((volatile uint32_t *) SPI_PUSHR_ADDR((a)))
#define SPI_PUSHR(a)      (*(SPI_PUSHR_PTR((a))))

#define SPI_POPR_ADDR(a) (a + SPI_POPR_OFFSET)
#define SPI_POPR_BB(a,b) (BIT_2_BB2_ADDR((SPI_POPR_ADDR((a))),(b)))
#define SPI_POPR_PTR(a)  ((volatile uint32_t *) SPI_POPR_ADDR((a)))
#define SPI_POPR(a)      (*(SPI_POPR_PTR((a))))

#if 0
#define PIN_SIN      0
#define PIN_SOUT     1
#define PIN_SCK      2
#define PIN_PCS0     3
#define NUM_PINS     4
#endif

/*******************************************************************************
* SDHC 
*******************************************************************************/
#define SDHC_BASE_ADDR 0x400B1000

#define SDHC_DSADDR_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x00))
#define SDHC_BLKATTR_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x04))
#define SDHC_CMDARG_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x08))
#define SDHC_XFERTYP_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x0C))
#define SDHC_CMDRSP_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x10))
#define SDHC_CMDRSP_REG_0     (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x10))
#define SDHC_CMDRSP_REG_1     (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x14))
#define SDHC_CMDRSP_REG_2     (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x18))
#define SDHC_CMDRSP_REG_3     (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x1C))
#define SDHC_DATPORT_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x20))
#define SDHC_PRSSTAT_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x24))
#define SDHC_PROCTL_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x28))
#define SDHC_SYSCTL_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x2C))
#define SDHC_IRQSTAT_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x30))
#define SDHC_IRQSTATEN_REG    (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x34))
#define SDHC_IRQSIGEN_REG     (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x38))
#define SDHC_AC12ERR_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x3C))
#define SDHC_HTCAPBLT_REG     (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x40))
#define SDHC_WML_REG          (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x44))
#define SDHC_FEVT_REG         (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x50))
#define SDHC_ADMAES_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x54))
#define SDHC_ADSADDR_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0x58))
#define SDHC_VENDOR_REG       (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0xC0))
#define SDHC_MMCBOOT_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0xC4))
#define SDHC_HOSTVER_REG      (*(volatile uint32_t *) (SDHC_BASE_ADDR + 0xFC))

/* PROCTL Bit Fields */
enum {
    SDHC_PROCTL_LCTL_MASK     = 1 << 0,
    SDHC_PROCTL_LCTL_SHIFT    = 0,
    SDHC_PROCTL_DTW_MASK      = 3 << 1,
    SDHC_PROCTL_DTW_SHIFT     = 1,
    SDHC_PROCTL_D3CD_MASK     = 1 << 3,
    SDHC_PROCTL_D3CD_SHIFT    = 3,
    SDHC_PROCTL_EMODE_MASK    = 3 << 4,
    SDHC_PROCTL_EMODE_SHIFT   = 4,
    SDHC_PROCTL_CDTL_MASK     = 1 << 6,
    SDHC_PROCTL_CDTL_SHIFT    = 6,
    SDHC_PROCTL_CDSS_MASK     = 1 << 7,
    SDHC_PROCTL_CDSS_SHIFT    = 7,
    SDHC_PROCTL_DMAS_MASK     = 3 << 8,
    SDHC_PROCTL_DMAS_SHIFT    = 8,
    SDHC_PROCTL_SABGREQ_MASK  = 1 << 16,
    SDHC_PROCTL_SABGREQ_SHIFT = 16,
    SDHC_PROCTL_CREQ_MASK     = 1 << 17,
    SDHC_PROCTL_CREQ_SHIFT    = 17,
    SDHC_PROCTL_RWCTL_MASK    = 1 << 18,
    SDHC_PROCTL_RWCTL_SHIFT   = 18,
    SDHC_PROCTL_IABG_MASK     = 1 << 19,
    SDHC_PROCTL_IABG_SHIFT    = 19,
    SDHC_PROCTL_WECINT_MASK   = 1 << 24,
    SDHC_PROCTL_WECINT_SHIFT  = 24,
    SDHC_PROCTL_WECINS_MASK   = 1 << 25,
    SDHC_PROCTL_WECINS_SHIFT  = 25,
    SDHC_PROCTL_WECRM_MASK    = 1 << 26,
    SDHC_PROCTL_WECRM_SHIFT   = 26,
};

/* PRSSTAT Bit Fields */
enum {
    SDHC_PRSSTAT_CIHB_MASK     = 1 << 0,
    SDHC_PRSSTAT_CIHB_SHIFT    = 0,
    SDHC_PRSSTAT_CDIHB_MASK    = 1 << 1,
    SDHC_PRSSTAT_CDIHB_SHIFT   = 1,
    SDHC_PRSSTAT_DLA_MASK      = 1 << 2,
    SDHC_PRSSTAT_DLA_SHIFT     = 2,
    SDHC_PRSSTAT_SDSTB_MASK    = 1 << 3,
    SDHC_PRSSTAT_SDSTB_SHIFT   = 3,
    SDHC_PRSSTAT_IPGOFF_MASK   = 1 << 4,
    SDHC_PRSSTAT_IPGOFF_SHIFT  = 4,
    SDHC_PRSSTAT_HCKOFF_MASK   = 1 << 5,
    SDHC_PRSSTAT_HCKOFF_SHIFT  = 5,
    SDHC_PRSSTAT_PEROFF_MASK   = 1 << 6,
    SDHC_PRSSTAT_PEROFF_SHIFT  = 6,
    SDHC_PRSSTAT_SDOFF_MASK    = 1 << 7,
    SDHC_PRSSTAT_SDOFF_SHIFT   = 7,
    SDHC_PRSSTAT_WTA_MASK      = 1 << 8,
    SDHC_PRSSTAT_WTA_SHIFT     = 8,
    SDHC_PRSSTAT_RTA_MASK      = 1 << 9,
    SDHC_PRSSTAT_RTA_SHIFT     = 9,
    SDHC_PRSSTAT_BWEN_MASK     = 1 << 10,
    SDHC_PRSSTAT_BWEN_SHIFT    = 10,
    SDHC_PRSSTAT_BREN_MASK     = 1 << 11,
    SDHC_PRSSTAT_BREN_SHIFT    = 11,
    SDHC_PRSSTAT_CINS_MASK     = 1 << 16,
    SDHC_PRSSTAT_CINS_SHIFT    = 16,
    SDHC_PRSSTAT_CLSL_MASK     = 1 << 23,
    SDHC_PRSSTAT_CLSL_SHIFT    = 23,
    SDHC_PRSSTAT_DLSL_MASK     = 0xFF << 24,
    SDHC_PRSSTAT_DLSL_SHIFT    = 24,
};

enum {
    SDHC_XFERTYP_DMAEN_MASK    = 1 << 0,
    SDHC_XFERTYP_DMAEN_SHIFT   = 0,
    SDHC_XFERTYP_BCEN_MASK     = 1 << 1,
    SDHC_XFERTYP_BCEN_SHIFT    = 1,
    SDHC_XFERTYP_AC12EN_MASK   = 1 << 2,
    SDHC_XFERTYP_AC12EN_SHIFT  = 2,
    SDHC_XFERTYP_DTDSEL_MASK   = 1 << 4,
    SDHC_XFERTYP_DTDSEL_SHIFT  = 4,
    SDHC_XFERTYP_MSBSEL_MASK   = 1 << 5,
    SDHC_XFERTYP_MSBSEL_SHIFT  = 5,
    SDHC_XFERTYP_RSPTYP_MASK   = 3 << 16,
    SDHC_XFERTYP_RSPTYP_SHIFT  = 16,
    SDHC_XFERTYP_CCCEN_MASK    = 1 << 19,
    SDHC_XFERTYP_CCCEN_SHIFT   = 19,
    SDHC_XFERTYP_CICEN_MASK    = 1 << 20,
    SDHC_XFERTYP_CICEN_SHIFT   = 20,
    SDHC_XFERTYP_DPSEL_MASK    = 1 << 21,
    SDHC_XFERTYP_DPSEL_SHIFT   = 21,
    SDHC_XFERTYP_CMDTYP_MASK   = 3 << 22,
    SDHC_XFERTYP_CMDTYP_SHIFT  = 22,
    SDHC_XFERTYP_CMDINX_MASK   = 0x3F << 24,
    SDHC_XFERTYP_CMDINX_SHIFT  = 24,
};

/* CMDARG Bit Fields */
enum {
    SDHC_CMDARG_CMDARG_MASK    = 0xFFFFFFFF,
    SDHC_CMDARG_CMDARG_SHIFT   = 0,
};

/* CMDRSP Bit Fields */
enum {
    SDHC_CMDRSP_CMDRSP0_MASK   = 0xFFFFFFFF,
    SDHC_CMDRSP_CMDRSP0_SHIFT  = 0,
    SDHC_CMDRSP_CMDRSP1_MASK   = 0xFFFFFFFF,
    SDHC_CMDRSP_CMDRSP1_SHIFT  = 0,
    SDHC_CMDRSP_CMDRSP2_MASK   = 0xFFFFFFFF,
    SDHC_CMDRSP_CMDRSP2_SHIFT  = 0,
    SDHC_CMDRSP_CMDRSP3_MASK   = 0xFFFFFFFF,
    SDHC_CMDRSP_CMDRSP3_SHIFT  = 0,
};

/* WML Bit Fields */
enum {
    SDHC_WML_RDWML_MASK        = 0xFF,
    SDHC_WML_RDWML_SHIFT       = 0,
    SDHC_WML_WRWML_MASK        = 0xFF0000,
    SDHC_WML_WRWML_SHIFT       = 16,
    SDHC_WML_WRBRSTLEN_MASK    = 0x1F000000,
    SDHC_WML_WRBRSTLEN_SHIFT   = 24,
};


enum {
    SDHC_IRQSTAT_CC_MASK          = 1 << 0,
    SDHC_IRQSTAT_CC_SHIFT         = 0,
    SDHC_IRQSTAT_TC_MASK          = 1 << 1,
    SDHC_IRQSTAT_TC_SHIFT         = 1,
    SDHC_IRQSTAT_BGE_MASK         = 1 << 2,
    SDHC_IRQSTAT_BGE_SHIFT        = 2,
    SDHC_IRQSTAT_DINT_MASK        = 1 << 3,
    SDHC_IRQSTAT_DINT_SHIFT       = 3,
    SDHC_IRQSTAT_BWR_MASK         = 1 << 4,
    SDHC_IRQSTAT_BWR_SHIFT        = 4,
    SDHC_IRQSTAT_BRR_MASK         = 1 << 5,
    SDHC_IRQSTAT_BRR_SHIFT        = 5,
    SDHC_IRQSTAT_CINS_MASK        = 1 << 6,
    SDHC_IRQSTAT_CINS_SHIFT       = 6,
    SDHC_IRQSTAT_CRM_MASK         = 1 << 7,
    SDHC_IRQSTAT_CRM_SHIFT        = 7,
    SDHC_IRQSTAT_CINT_MASK        = 1 << 8,
    SDHC_IRQSTAT_CINT_SHIFT       = 8,
    SDHC_IRQSTAT_CTOE_MASK        = 1 << 16,
    SDHC_IRQSTAT_CTOE_SHIFT       = 16,
    SDHC_IRQSTAT_CCE_MASK         = 1 << 17,
    SDHC_IRQSTAT_CCE_SHIFT        = 17,
    SDHC_IRQSTAT_CEBE_MASK        = 1 << 18,
    SDHC_IRQSTAT_CEBE_SHIFT       = 18,
    SDHC_IRQSTAT_CIE_MASK         = 1 << 19,
    SDHC_IRQSTAT_CIE_SHIFT        = 19,
    SDHC_IRQSTAT_DTOE_MASK        = 1 << 20,
    SDHC_IRQSTAT_DTOE_SHIFT       = 20,
    SDHC_IRQSTAT_DCE_MASK         = 1 << 21,
    SDHC_IRQSTAT_DCE_SHIFT        = 21,
    SDHC_IRQSTAT_DEBE_MASK        = 1 << 22,
    SDHC_IRQSTAT_DEBE_SHIFT       = 22,
    SDHC_IRQSTAT_AC12E_MASK       = 1 << 24,
    SDHC_IRQSTAT_AC12E_SHIFT      = 24,
    SDHC_IRQSTAT_DMAE_MASK        = 1 << 28,
    SDHC_IRQSTAT_DMAE_SHIFT       = 28,
};

enum {
    SDHC_IRQSTATEN_CCSEN_MASK     = 1 << 0,
    SDHC_IRQSTATEN_CCSEN_SHIFT    = 0,
    SDHC_IRQSTATEN_TCSEN_MASK     = 1 << 1,
    SDHC_IRQSTATEN_TCSEN_SHIFT    = 1,
    SDHC_IRQSTATEN_BGESEN_MASK    = 1 << 2,
    SDHC_IRQSTATEN_BGESEN_SHIFT   = 2,
    SDHC_IRQSTATEN_DINTSEN_MASK   = 1 << 3,
    SDHC_IRQSTATEN_DINTSEN_SHIFT  = 3,
    SDHC_IRQSTATEN_BWRSEN_MASK    = 1 << 4,
    SDHC_IRQSTATEN_BWRSEN_SHIFT   = 4,
    SDHC_IRQSTATEN_BRRSEN_MASK    = 1 << 5,
    SDHC_IRQSTATEN_BRRSEN_SHIFT   = 5,
    SDHC_IRQSTATEN_CINSEN_MASK    = 1 << 6,
    SDHC_IRQSTATEN_CINSEN_SHIFT   = 6,
    SDHC_IRQSTATEN_CRMSEN_MASK    = 1 << 7,
    SDHC_IRQSTATEN_CRMSEN_SHIFT   = 7,
    SDHC_IRQSTATEN_CINTSEN_MASK   = 1 << 8,
    SDHC_IRQSTATEN_CINTSEN_SHIFT  = 8,
    SDHC_IRQSTATEN_CTOESEN_MASK   = 1 << 16,
    SDHC_IRQSTATEN_CTOESEN_SHIFT  = 16,
    SDHC_IRQSTATEN_CCESEN_MASK    = 1 << 17,
    SDHC_IRQSTATEN_CCESEN_SHIFT   = 17,
    SDHC_IRQSTATEN_CEBESEN_MASK   = 1 << 18,
    SDHC_IRQSTATEN_CEBESEN_SHIFT  = 18,
    SDHC_IRQSTATEN_CIESEN_MASK    = 1 << 19,
    SDHC_IRQSTATEN_CIESEN_SHIFT   = 19,
    SDHC_IRQSTATEN_DTOESEN_MASK   = 1 << 20,
    SDHC_IRQSTATEN_DTOESEN_SHIFT  = 20,
    SDHC_IRQSTATEN_DCESEN_MASK    = 1 << 21,
    SDHC_IRQSTATEN_DCESEN_SHIFT   = 21,
    SDHC_IRQSTATEN_DEBESEN_MASK   = 1 << 22,
    SDHC_IRQSTATEN_DEBESEN_SHIFT  = 22,
    SDHC_IRQSTATEN_AC12ESEN_MASK  = 1 << 24,
    SDHC_IRQSTATEN_AC12ESEN_SHIFT = 24,
    SDHC_IRQSTATEN_DMAESEN_MASK   = 1 << 28,
    SDHC_IRQSTATEN_DMAESEN_SHIFT  = 28,
};

/* SYSCTL Bit Fields */
enum {
    SDHC_SYSCTL_IPGEN_MASK        = 1 << 0,
    SDHC_SYSCTL_IPGEN_SHIFT       = 0,
    SDHC_SYSCTL_HCKEN_MASK        = 1 << 1,
    SDHC_SYSCTL_HCKEN_SHIFT       = 1,
    SDHC_SYSCTL_PEREN_MASK        = 1 << 2,
    SDHC_SYSCTL_PEREN_SHIFT       = 2,
    SDHC_SYSCTL_SDCLKEN_MASK      = 1 << 3,
    SDHC_SYSCTL_SDCLKEN_SHIFT     = 3,
    SDHC_SYSCTL_DVS_MASK          = 0xF << 4,
    SDHC_SYSCTL_DVS_SHIFT         = 4,
    SDHC_SYSCTL_SDCLKFS_MASK      = 0xFF << 8,
    SDHC_SYSCTL_SDCLKFS_SHIFT     = 8,
    SDHC_SYSCTL_DTOCV_MASK        = 0xF << 16,
    SDHC_SYSCTL_DTOCV_SHIFT       = 16,
    SDHC_SYSCTL_RSTA_MASK         = 1 << 24,
    SDHC_SYSCTL_RSTA_SHIFT        = 24,
    SDHC_SYSCTL_RSTC_MASK         = 1 << 25,
    SDHC_SYSCTL_RSTC_SHIFT        = 25,
    SDHC_SYSCTL_RSTD_MASK         = 1 << 26,
    SDHC_SYSCTL_RSTD_SHIFT        = 26,
    SDHC_SYSCTL_INITA_MASK        = 1 << 27,
    SDHC_SYSCTL_INITA_SHIFT       = 27,
};

/* BLKATTR Bit Fields */
enum {
    SDHC_BLKATTR_BLKSIZE_MASK     = 0x1FFF,
    SDHC_BLKATTR_BLKSIZE_SHIFT    = 0,
    SDHC_BLKATTR_BLKCNT_MASK      = 0xFFFF0000,
    SDHC_BLKATTR_BLKCNT_SHIFT     = 16,
};


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
    FTFL_CMD_SWAP               = 0x46,
    FTFL_CMD_PRGRM_PARTITION    = 0x80,
    FTFL_CMD_SET_FLEXRAM_FN     = 0x81,
} ftflFCMD_t;

#define FTFL_FLASH_SECTOR_SIZE 0x800   /* 2   KBytes */
#define FTFL_FLASH_BLOCK_SIZE  0x40000 /* 256 KBytes */

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
typedef enum {
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
    TSI_SCANC_AMCLKS_MASK     = 3 << 3,    /* R/W - 0 */
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

/* CRC Data Register */
/* NOTE: Datasheet references it as CRC_CRC but I call it CRC_DATA */
/* Contains the value of the seed, data and the checksum */
#define CRC_DATA_ADDR      0x40032000
#define CRC_DATA_PTR       ((volatile uint32_t *) CRC_DATA_ADDR)
#define CRC_DATA           (*(CRC_DATA_PTR))
                                                        /* 8/16 Bit Access */
#define CRC_DATA_16BIT_PTR ((volatile uint16_t *) CRC_DATA_ADDR)
#define CRC_DATA_16BIT     (*(CRC_DATA_16BIT_PTR))
#define CRC_DATA_8BIT_PTR  ((volatile uint8_t  *) CRC_DATA_ADDR)
#define CRC_DATA_8BIT      (*(CRC_DATA_8BIT_PTR))

/* CRC Polynomial Register */
/* Contains the value of the polynomial for the CRC calc */
#define CRC_GPOLY_ADDR      0x40032004
#define CRC_GPOLY_PTR       ((volatile uint32_t *) CRC_GPOLY_ADDR)
#define CRC_GPOLY           (*(CRC_GPOLY_PTR))
                                                             /*16 Bit Access */
#define CRC_GPOLY_16BIT_PTR ((volatile uint16_t *) CRC_GPOLY_ADDR)
#define CRC_GPOLY_16BIT     (*(CRC_GPOLY_16BIT_PTR))

/* CRC Control Register */
/* Controls the configuration of the CRC module */
#define CRC_CTRL_ADDR      0x40032008
#define CRC_CTRL_PTR       ( (volatile uint32_t *) CRC_CTRL_ADDR)
#define CRC_CTRL           (*(CRC_CTRL_PTR))
#define CRC_CTRL_BB_ADDR   (MEMMAP_BB_GPIO_PBRIDGE_ADDR + (0x32008 * 32U))
                                                              /* Bit Numbers */
#define CRC_CTRL_TOT1_BIT      31U
#define CRC_CTRL_TOT0_BIT      30U
#define CRC_CTRL_TOT_BIT       (CRC_CTRL_TOT0_BIT)
#define CRC_CTRL_TOTR1_BIT     29U
#define CRC_CTRL_TOTR0_BIT     28U
#define CRC_CTRL_TOTR_BIT      (CRC_CTRL_TOTR0_BIT)
#define CRC_CTRL_FXOR_BIT      26U
#define CRC_CTRL_WAS_BIT       25U
#define CRC_CTRL_TCRC_BIT      24U
                                                                    /* Masks */
#define CRC_CTRL_TOT1_MSK      (1U<<CRC_CTRL_TOT1_BIT)
#define CRC_CTRL_TOT0_MSK      (1U<<CRC_CTRL_TOT0_BIT)
#define CRC_CTRL_TOT_MSK       (CRC_CTRL_TOT1_MSK | CRC_CTRL_TOT0_MSK)
#define CRC_CTRL_TOTR1_MSK     (1U<<CRC_CTRL_TOTR1_BIT)
#define CRC_CTRL_TOTR0_MSK     (1U<<CRC_CTRL_TOTR0_BIT)
#define CRC_CTRL_TOTR_MSK      (CRC_CTRL_TOTR1_MSK | CRC_CTRL_TOTR0_MSK)
#define CRC_CTRL_FXOR_MSK      (1U<<CRC_CTRL_FXOR_BIT)
#define CRC_CTRL_WAS_MSK       (1U<<CRC_CTRL_WAS_BIT)
#define CRC_CTRL_TCRC_MSK      (1U<<CRC_CTRL_TCRC_BIT)
                                                     /* Bit-Banded Addresses */
#define CRC_CTRL_TOT1_BB_ADDR  (CRC_CTRL_BB_ADDR +(CRC_CTRL_TOT1_BIT  * 4U))
#define CRC_CTRL_TOT0_BB_ADDR  (CRC_CTRL_BB_ADDR +(CRC_CTRL_TOT0_BIT  * 4U))
#define CRC_CTRL_TOTR1_BB_ADDR (CRC_CTRL_BB_ADDR +(CRC_CTRL_TOTR1_BIT * 4U))
#define CRC_CTRL_TOTR0_BB_ADDR (CRC_CTRL_BB_ADDR +(CRC_CTRL_TOTR0_BIT * 4U))
#define CRC_CTRL_FXOR_BB_ADDR  (CRC_CTRL_BB_ADDR +(CRC_CTRL_FXOR_BIT  * 4U))
#define CRC_CTRL_WAS_BB_ADDR   (CRC_CTRL_BB_ADDR +(CRC_CTRL_WAS_BIT   * 4U))
#define CRC_CTRL_TCRC_BB_ADDR  (CRC_CTRL_BB_ADDR +(CRC_CTRL_TCRC_BIT  * 4U))
                                                      /* Bit-Banded Pointers */
#define CRC_CTRL_TOT1_BB_PTR   ((volatile uint32_t *) CRC_CTRL_TOT1_BB_ADDR)
#define CRC_CTRL_TOT0_BB_PTR   ((volatile uint32_t *) CRC_CTRL_TOT0_BB_ADDR)
#define CRC_CTRL_TOTR1_BB_PTR  ((volatile uint32_t *) CRC_CTRL_TOTR0_BB_ADDR)
#define CRC_CTRL_TOTR0_BB_PTR  ((volatile uint32_t *) CRC_CTRL_TOTR1_BB_ADDR)
#define CRC_CTRL_FXOR_BB_PTR   ((volatile uint32_t *) CRC_CTRL_FXOR_BB_ADDR)
#define CRC_CTRL_WAS_BB_PTR    ((volatile uint32_t *) CRC_CTRL_WAS_BB_ADDR)
#define CRC_CTRL_TCRC_BB_PTR   ((volatile uint32_t *) CRC_CTRL_TCRC_BB_ADDR)
                                         /* Bit-Banded Dereferenced Pointers */
#define CRC_CTRL_TOT1_BB       (*(CRC_CTRL_TOT1_BB_PTR))
#define CRC_CTRL_TOT0_BB       (*(CRC_CTRL_TOT0_BB_PTR))
#define CRC_CTRL_TOTR1_BB      (*(CRC_CTRL_TOTR1_BB_PTR))
#define CRC_CTRL_TOTR0_BB      (*(CRC_CTRL_TOTR0_BB_PTR))
#define CRC_CTRL_FXOR_BB       (*(CRC_CTRL_FXOR_BB_PTR))
#define CRC_CTRL_WAS_BB        (*(CRC_CTRL_WAS_BB_PTR))
#define CRC_CTRL_TCRC_BB       (*(CRC_CTRL_TCRC_BB_PTR))

/*******************************************************************************
* MPU
*******************************************************************************/

typedef enum {
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
* FTM
*******************************************************************************/

#define FTM_BASE_ADDR 0x40038000

typedef struct {
    uint32_t sc;      /* Status & control                     R/W, 0x00000000 */
    uint32_t cnt;     /* Counter                              R/W, 0x00000000 */
    uint32_t mod;     /* Modulo                               R/W, 0x00000000 */
    uint32_t c0cs;    /* Channel 0 status & control           R/W, 0x00000000 */
    uint32_t c0v;     /* Channel 0 value                      R/W, 0x00000000 */
    uint32_t c1cs;    /* Channel 1 status & control           R/W, 0x00000000 */
    uint32_t c1v;     /* Channel 1 value                      R/W, 0x00000000 */
    uint32_t c2cs;    /* Channel 2 status & control           R/W, 0x00000000 */
    uint32_t c2v;     /* Channel 2 value                      R/W, 0x00000000 */
    uint32_t c3cs;    /* Channel 3 status & control           R/W, 0x00000000 */
    uint32_t c3v;     /* Channel 3 value                      R/W, 0x00000000 */
    uint32_t c4cs;    /* Channel 4 status & control           R/W, 0x00000000 */
    uint32_t c4v;     /* Channel 4 value                      R/W, 0x00000000 */
    uint32_t c5cs;    /* Channel 5 status & control           R/W, 0x00000000 */
    uint32_t c5v;     /* Channel 5 value                      R/W, 0x00000000 */
    uint32_t c6cs;    /* Channel 6 status & control           R/W, 0x00000000 */
    uint32_t c6v;     /* Channel 6 value                      R/W, 0x00000000 */
    uint32_t c7cs;    /* Channel 7 status & control           R/W, 0x00000000 */
    uint32_t c7v;     /* Channel 7 value                      R/W, 0x00000000 */
    uint32_t cntin;   /* Counter initial value                R/W, 0x00000000 */
    uint32_t status;  /* Capture and compare status           R/W, 0x00000000 */
    uint32_t mode;    /* Features mode selection              R/W, 0x00000000 */
    uint32_t sync;    /* Synchronization                      R/W, 0x00000000 */
    uint32_t outinit; /* Initial State for channels output    R/W, 0x00000000 */
    uint32_t outmask; /* Output mask                          R/W, 0x00000000 */
    uint32_t combine; /* Function for linked channels         R/W, 0x00000000 */
    uint32_t deadtime;/* Deadtime insertion control           R/W, 0x00000000 */
    uint32_t exttrig; /* FTM external trigger                 R/W, 0x00000000 */
    uint32_t pol;     /* Channels Polishness                  R/W, 0x00000000 */
    uint32_t fms;     /* Fault Mode Status                    R/W, 0x00000000 */
    uint32_t filter;  /* Inpute capture filter control        R/W, 0x00000000 */
    uint32_t fltctrl; /* Fault control                        R/W, 0x00000000 */
    uint32_t qdctrl;  /* Quadrature decoder control & status  R/W, 0x00000000 */
    uint32_t conf;    /* Configuration                        R/W, 0x00000000 */
    uint32_t fltpol;  /* FTM Fault Input Polarity             R/W, 0x00000000 */
    uint32_t synconf; /* Synchronization Configuration        R/W, 0x00000000 */
    uint32_t invctrl; /* FTM inverting control                R/W, 0x00000000 */
    uint32_t swoctrl; /* FTM software output control          R/W, 0x00000000 */
    uint32_t pwmload; /* FTM PWM Load                         R/W, 0x00000000 */
    uint32_t reserved[(0xf68/4) - 1];
} ftm_t;

enum {
    FTM_0,
    FTM_1,
    FTM_2,

    MAX_FTM,
};

typedef struct {
    ftm_t ftm[3];
} ftmCtrl_t;

extern volatile ftmCtrl_t * const ftmCtrl;



/* FTMx_SC */
enum {
    FTM_SC_TOF_BIT   = 1 << 7,
    FTM_SC_TOIE_BIT  = 1 << 6,
    FTM_SC_CPWMS_BIT = 1 << 5,
};
#define FTM_SC_CLKS_MASK (0x03)
#define FTM_SC_CLKS_SHIFT 3
#define FTM_SC_PS_MASK   (0x07)
enum {
    FTM_SC_CLKS_NONE,
    FTM_SC_CLKS_BUS,
    FTM_SC_CLKS_FIXED_FREQ, /* Fixed freq is MCGFFCLK on K60N512 */
    FMT_SC_CLKS_EXTERNAL_CLOCK,
};
enum {
    FTM_SC_PS_1,
    FTM_SC_PS_2,
    FTM_SC_PS_4,
    FTM_SC_PS_8,
    FTM_SC_PS_16,
    FTM_SC_PS_32,
    FTM_SC_PS_64,
    FTM_SC_PS_128,
};

/* FTMx_CNT */
#define FMT_CNT_COUNT_MASK 0xFFFF

/* FTMx_MOD */
#define FMT_CNT_MOD_MASK 0xFFFF

/* FTMx_CnSC */
enum {
    FTM_CH_CS_CHF_BIT   = 1 << 7,
    FTM_CH_CS_CHIE_BIT  = 1 << 6,
    FTM_CH_CS_MSA_BIT   = 1 << 4,
    FTM_CH_CS_ELSB_BIT  = 1 << 3,
    FTM_CH_CS_ELSA_BIT  = 1 << 2,
    /* Reserved 1 << 1 */
    FTM_CH_CS_DMA_BIT   = 1 << 0,
};

/* FTMx_CV */
#define FMT_CH_VALUE_MASK 0xFFFF

/* FTMx_STATUS */
enum {
    FTM_STATUS_CH7_EVENT_BIT    = 1 << 7,
    FTM_STATUS_CH6_EVENT_BIT    = 1 << 6,
    FTM_STATUS_CH5_EVENT_BIT    = 1 << 5,
    FTM_STATUS_CH4_EVENT_BIT    = 1 << 4,
    FTM_STATUS_CH3_EVENT_BIT    = 1 << 3,
    FTM_STATUS_CH2_EVENT_BIT    = 1 << 2,
    FTM_STATUS_CH1_EVENT_BIT    = 1 << 1,
    FTM_STATUS_CH0_EVENT_BIT    = 1 << 0,
};

/* FTMx_MODE */
enum {
    FTM_MODE_FAULT_IE_BIT       = 1 << 7,
    FTM_MODE_CAPTTEST_BIT       = 1 << 4,
    FTM_MODE_PWMSYNC_BIT        = 1 << 3,
    FTM_MODE_WPDIS_BIT          = 1 << 2,
    FTM_MODE_INIT_BIT           = 1 << 1,
    FTM_MODE_FTMEN_BIT          = 1 << 0,
};
#define FMT_MODE_FAULT_MODE_MASK  0x3
#define FMT_MODE_FAULT_MODE_SHIFT 5
enum {
    FTM_MODE_FAULT_MODE_DISABLED,
    FTM_MODE_FAULT_MODE_EVEN_CHANNELS,
    FTM_MODE_FAULT_MODE_ALL_CHANNELS,
    FTM_MODE_FAULT_MODE_ALL_CHANNELS_AUTO_CLEAR,
};

/* FTMx_SYNC */
enum {
    FTM_SYNC_SWSYNC_BIT         = 1 << 7,
    FTM_SYNC_TRIG2_BIT          = 1 << 6,
    FTM_SYNC_TRIG1_BIT          = 1 << 5,
    FTM_SYNC_TRIG0_BIT          = 1 << 4,
    FTM_SYNC_SYNCHOM_BIT        = 1 << 3,
    FTM_SYNC_REINT_BIT          = 1 << 2,
    FTM_SYNC_CNTMAX_BIT         = 1 << 1,
    FTM_SYNC_CNTMIN_BIT         = 1 << 0,
};

/* FTMx_OUTINIT */
enum {
    FTM_OUTINIT_CH7_OI_BIT      = 1 << 7,
    FTM_OUTINIT_CH6_OI_BIT      = 1 << 6,
    FTM_OUTINIT_CH5_OI_BIT      = 1 << 5,
    FTM_OUTINIT_CH4_OI_BIT      = 1 << 4,
    FTM_OUTINIT_CH3_OI_BIT      = 1 << 3,
    FTM_OUTINIT_CH2_OI_BIT      = 1 << 2,
    FTM_OUTINIT_CH1_OI_BIT      = 1 << 1,
    FTM_OUTINIT_CH0_OI_BIT      = 1 << 0,
};

/* FTMx_OUTMASK */
enum {
    FTM_OUTMASK_CH7_OI_BIT      = 1 << 7,
    FTM_OUTMASK_CH6_OI_BIT      = 1 << 6,
    FTM_OUTMASK_CH5_OI_BIT      = 1 << 5,
    FTM_OUTMASK_CH4_OI_BIT      = 1 << 4,
    FTM_OUTMASK_CH3_OI_BIT      = 1 << 3,
    FTM_OUTMASK_CH2_OI_BIT      = 1 << 2,
    FTM_OUTMASK_CH1_OI_BIT      = 1 << 1,
    FTM_OUTMASK_CH0_OI_BIT      = 1 << 0,
};

/* FTMx_COMBINE */
enum {
    FTM_COMBINE_CH6_CH7_FAULTEN_BIT  = 1 << 30,
    FTM_COMBINE_CH6_CH7_SYNCEN_BIT   = 1 << 29,
    FTM_COMBINE_CH6_CH7_DTEN_BIT     = 1 << 28,
    FTM_COMBINE_CH6_CH7_DECAP_BIT    = 1 << 27,
    FTM_COMBINE_CH6_CH7_DECAPEN_BIT  = 1 << 26,
    FTM_COMBINE_CH6_CH7_COMP_BIT     = 1 << 25,
    FTM_COMBINE_CH6_CH7_COMBINE_BIT  = 1 << 24,

    FTM_COMBINE_CH4_CH5_FAULTEN_BIT  = 1 << 22,
    FTM_COMBINE_CH4_CH5_SYNCEN_BIT   = 1 << 21,
    FTM_COMBINE_CH4_CH5_DTEN_BIT     = 1 << 20,
    FTM_COMBINE_CH4_CH5_DECAP_BIT    = 1 << 19,
    FTM_COMBINE_CH4_CH5_DECAPEN_BIT  = 1 << 18,
    FTM_COMBINE_CH4_CH5_COMP_BIT     = 1 << 17,
    FTM_COMBINE_CH4_CH5_COMBINE_BIT  = 1 << 16,

    FTM_COMBINE_CH2_CH3_FAULTEN_BIT  = 1 << 14,
    FTM_COMBINE_CH2_CH3_SYNCEN_BIT   = 1 << 13,
    FTM_COMBINE_CH2_CH3_DTEN_BIT     = 1 << 12,
    FTM_COMBINE_CH2_CH3_DECAP_BIT    = 1 << 11,
    FTM_COMBINE_CH2_CH3_DECAPEN_BIT  = 1 << 10,
    FTM_COMBINE_CH2_CH3_COMP_BIT     = 1 << 9,
    FTM_COMBINE_CH2_CH3_COMBINE_BIT  = 1 << 8,

    FTM_COMBINE_CH0_CH1_FAULTEN_BIT  = 1 << 6,
    FTM_COMBINE_CH0_CH1_SYNCEN_BIT   = 1 << 5,
    FTM_COMBINE_CH0_CH1_DTEN_BIT     = 1 << 4,
    FTM_COMBINE_CH0_CH1_DECAP_BIT    = 1 << 3,
    FTM_COMBINE_CH0_CH1_DECAPEN_BIT  = 1 << 2,
    FTM_COMBINE_CH0_CH1_COMP_BIT     = 1 << 1,
    FTM_COMBINE_CH0_CH1_COMBINE_BIT  = 1 << 0,
};

/* FTMx_DEADTIME */
#define FTM_DEADTIME_DTPS_MASK 0x3
#define FTM_DEADTIME_DTPS_SHIFT 6
enum {
    FTM_DEADTIME_DTPS_1  = 0,

    FTM_DEADTIME_DTPS_4  = 2,
    FTM_DEADTIME_DTPS_16 = 3,
};
#define FTM_DEADTIME_DTVAL_MAX_VALUE 0x3F /* Limit value */


/* FTMx_EXTTRIG */
enum {
    FTM_EXTTRIG_TRIGF_BIT          = 1 << 7,
    FTM_EXTTRIG_INIT_TRIG_EN_BIT   = 1 << 6,
    FTM_EXTTRIG_CH1_TRIG_BIT       = 1 << 5,
    FTM_EXTTRIG_CH0_TRIG_BIT       = 1 << 4,
    FTM_EXTTRIG_CH5_TRIG_BIT       = 1 << 3,
    FTM_EXTTRIG_CH4_TRIG_BIT       = 1 << 2,
    FTM_EXTTRIG_CH3_TRIG_BIT       = 1 << 1,
    FTM_EXTTRIG_CH2_TRIG_BIT       = 1 << 0,
};

/* FTMx_POL */
enum {
    FTM_POT_CH7_BIT     = 1 << 7,
    FTM_POT_CH6_BIT     = 1 << 6,
    FTM_POT_CH5_BIT     = 1 << 5,
    FTM_POT_CH4_BIT     = 1 << 4,
    FTM_POT_CH3_BIT     = 1 << 3,
    FTM_POT_CH2_BIT     = 1 << 2,
    FTM_POT_CH1_BIT     = 1 << 1,
    FTM_POT_CH0_BIT     = 1 << 0,
};

/* FTMx_FMS */
enum {
    FTM_FMS_FALUTF_BIT   = 1 << 7,
    FTM_FMS_WPEN_BIT     = 1 << 6,
    FTM_FMS_FALUTIN_BIT  = 1 << 5,

    FTM_FMS_FALUTF3_BIT  = 1 << 3,
    FTM_FMS_FALUTF2_BIT  = 1 << 2,
    FTM_FMS_FALUTF1_BIT  = 1 << 1,
    FTM_FMS_FALUTF0_BIT  = 1 << 0,
};

/* FTMx_FILTER */
#define FTM_FILTER_VALUE_MASK 0xF
#define FTM_FILTER_VALUE_CH3_SHIFT 12
#define FTM_FILTER_VALUE_CH2_SHIFT  8
#define FTM_FILTER_VALUE_CH1_SHIFT  4
#define FTM_FILTER_VALUE_CH0_SHIFT  0

/* FTMx_FLTCTRL */
#define FTM_FLTCTRL_FFAVL_MASK  0xF
#define FTM_FLTCTRL_FFAVL_SHIFT 8
enum {
    FTM_FLTCTRL_FFLTR3_EN_BIT   = 1 << 7,
    FTM_FLTCTRL_FFLTR2_EN_BIT   = 1 << 6,
    FTM_FLTCTRL_FFLTR1_EN_BIT   = 1 << 5,
    FTM_FLTCTRL_FFLTR0_EN_BIT   = 1 << 4,
    FTM_FLTCTRL_FAULT3_EN_BIT   = 1 << 3,
    FTM_FLTCTRL_FAULT2_EN_BIT   = 1 << 2,
    FTM_FLTCTRL_FAULT1_EN_BIT   = 1 << 1,
    FTM_FLTCTRL_FAULT0_EN_BIT   = 1 << 0,
};

/* FTMx_QDCTRL */
enum {
    FTM_QDCTRL_PHAFLTREN_BIT    = 1 << 7,
    FTM_QDCTRL_PHBFLTREN_BIT    = 1 << 6,
    FTM_QDCTRL_PHAPOL_BIT       = 1 << 5,
    FTM_QDCTRL_PHBPOL_BIT       = 1 << 4,
    FTM_QDCTRL_QUADMODE_BIT     = 1 << 3,
    FTM_QDCTRL_QUADIR_BIT       = 1 << 2,
    FTM_QDCTRL_TOFDIR_BIT       = 1 << 1,
    FTM_QDCTRL_QUADEN_BIT       = 1 << 0,
};

/* FTMx_CONF */
enum {
    FTM_CONF_GTBEOUT_BIT        = 1 << 10,
    FTM_CONF_GTBEEN_BIT         = 1 <<  9,
};
#define FTM_CONF_BDMMODE_MASK  0x3
#define FTM_CONF_BDMMODE_SHIFT 6

#define FTM_CONF_NUMTOF_MASK 0x1F
enum {
    FTM_CONF_NUMTOF_EACH_OVERFLOW,
    FTM_CONF_NUMTOF_1_IN_2_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_3_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_4_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_5_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_6_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_7_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_8_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_9_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_10_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_11_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_12_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_13_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_14_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_15_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_16_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_17_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_18_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_19_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_20_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_21_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_22_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_23_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_24_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_25_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_26_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_27_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_28_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_29_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_30_OVERFLOWS,
    FTM_CONF_NUMTOF_1_IN_31_OVERFLOWS,
};

/* FTMx_FTLPOL */
enum {
    FTM_FLTPOL_FLT3_POL_BIT     = 1 << 3,
    FTM_FLTPOL_FLT2_POL_BIT     = 1 << 2,
    FTM_FLTPOL_FLT1_POL_BIT     = 1 << 1,
};

/* FTMx_SYNCONF */
enum {
    FMT_SYNCONF_HWSOC_BIT       = 1 << 20,
    FMT_SYNCONF_HWINVC_BIT      = 1 << 19,
    FMT_SYNCONF_HWOM_BIT        = 1 << 18,
    FMT_SYNCONF_HWWRBUF_BIT     = 1 << 17,
    FMT_SYNCONF_HWRSTCNT_BIT    = 1 << 16,

    FMT_SYNCONF_SWSOC_BIT       = 1 << 12,
    FMT_SYNCONF_SWINVC_BIT      = 1 << 11,
    FMT_SYNCONF_SWOM_BIT        = 1 << 10,
    FMT_SYNCONF_SWWRBUF_BIT     = 1 <<  9,
    FMT_SYNCONF_SWRSTCNT_BIT    = 1 <<  8,
    FMT_SYNCONF_SYNCMODE_BIT    = 1 <<  7,

    FMT_SYNCONF_SWOC_BIT        = 1 <<  5,
    FMT_SYNCONF_INVC_BIT        = 1 <<  4,

    FMT_SYNCONF_CNTINC_BIT      = 1 <<  2,

    FMT_SYNCONF_HWTRIGMODE_BIT  = 1 <<  0,
};

/* FTMx_INVCTRL */
enum {
    FTM_INVCTRL_INV3EN_BIT      = 1 << 3,
    FTM_INVCTRL_INV2EN_BIT      = 1 << 2,
    FTM_INVCTRL_INV1EN_BIT      = 1 << 1,
    FTM_INVCTRL_INV0EN_BIT      = 1 << 0,
};

/* FTMx_SWOCTRL */
enum {
    FTM_SWOCVTRL_CH7_OCV_BIT    = 1 << 15,
    FTM_SWOCVTRL_CH6_OCV_BIT    = 1 << 14,
    FTM_SWOCVTRL_CH5_OCV_BIT    = 1 << 13,
    FTM_SWOCVTRL_CH4_OCV_BIT    = 1 << 12,
    FTM_SWOCVTRL_CH3_OCV_BIT    = 1 << 11,
    FTM_SWOCVTRL_CH2_OCV_BIT    = 1 << 10,
    FTM_SWOCVTRL_CH1_OCV_BIT    = 1 <<  9,
    FTM_SWOCVTRL_CH0_OCV_BIT    = 1 <<  8,

    FTM_SWOCTRL_CH7_OC_BIT      = 1 <<  7,
    FTM_SWOCTRL_CH6_OC_BIT      = 1 <<  6,
    FTM_SWOCTRL_CH5_OC_BIT      = 1 <<  5,
    FTM_SWOCTRL_CH4_OC_BIT      = 1 <<  4,
    FTM_SWOCTRL_CH3_OC_BIT      = 1 <<  3,
    FTM_SWOCTRL_CH2_OC_BIT      = 1 <<  2,
    FTM_SWOCTRL_CH1_OC_BIT      = 1 <<  1,
    FTM_SWOCTRL_CH0_OC_BIT      = 1 <<  0,
};

/* FMTx_PWMLOAD */
enum {
    FTM_PWMLOAD_LDOK_BIT        = 1 << 9,

    FTM_PWMLOAD_CH7SEL_BIT      = 1 << 7,
    FTM_PWMLOAD_CH6SEL_BIT      = 1 << 6,
    FTM_PWMLOAD_CH5SEL_BIT      = 1 << 5,
    FTM_PWMLOAD_CH4SEL_BIT      = 1 << 4,
    FTM_PWMLOAD_CH3SEL_BIT      = 1 << 3,
    FTM_PWMLOAD_CH2SEL_BIT      = 1 << 2,
    FTM_PWMLOAD_CH1SEL_BIT      = 1 << 1,
    FTM_PWMLOAD_CH0SEL_BIT      = 1 << 0,
};

/*******************************************************************************
* CLOCKS
*
* MCG     = Multipurpose Clock Generator
* OSC     = Oscillator
* RTC     = Real Time Clock
*
*******************************************************************************/

typedef enum {
    CLOCK_MCGIRCLK,
    CLOCK_MCGFFCLK,
    CLOCK_SYSTEM,                              /* Clocks the ARM Cortex M4 core */
    CLOCK_CORE = CLOCK_SYSTEM,
    CLOCK_BUS,
    CLOCK_FLEXBUS,
    CLOCK_FLASH,
    CLOCK_MCGPLLCLK,
    CLOCK_MCGFLLCLK,
    CLOCK_OSCERCLK,
    CLOCK_ERCLK32K,
    CLOCK_LPO,
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

/*******************************************************************************
* DMA MUX
*******************************************************************************/

#define DMAMUX_ENBL_BIT         7U
#define DMAMUX_TRIG_BIT         6U

#define DMAMUX_ENBL_MSK         (1U<<DMAMUX_ENBL_BIT)
#define DMAMUX_TRIG_MSK         (1U<<DMAMUX_TRIG_BIT)
#define DMAMUX_SOURCE_MSK       (0x3F)

#define DMAMUX_SOURCE_UART0RX   2U
#define DMAMUX_SOURCE_UART0TX   3U
#define DMAMUX_SOURCE_UART1RX   4U
#define DMAMUX_SOURCE_UART1TX   5U
#define DMAMUX_SOURCE_UART2RX   6U
#define DMAMUX_SOURCE_UART2TX   7U
#define DMAMUX_SOURCE_UART3RX   8U
#define DMAMUX_SOURCE_UART3TX   9U
#define DMAMUX_SOURCE_UART4RX   10U
#define DMAMUX_SOURCE_UART4TX   11U
#define DMAMUX_SOURCE_UART5RX   12U
#define DMAMUX_SOURCE_UART5TX   13U
#define DMAMUX_SOURCE_I2S0      14U
#define DMAMUX_SOURCE_I2S1      15U
#define DMAMUX_SOURCE_SPI0RX    16U
#define DMAMUX_SOURCE_SPI0TX    17U
#define DMAMUX_SOURCE_SPI1RX    18U
#define DMAMUX_SOURCE_SPI1TX    19U
/* TODO: Confirm that SPI2 works, they are strangly missing
 * in the datasheet. */
#define DMAMUX_SOURCE_SPI2RX    20U
#define DMAMUX_SOURCE_SPI2TX    21U
/* TODO: Finish this list later, its on page 92 */

#define DMAMUX_ADDR      0x40021000
#define DMAMUX_CHAN0    0x0
#define DMAMUX_CHAN1    0x1
#define DMAMUX_CHAN2    0x2
#define DMAMUX_CHAN3    0x3
#define DMAMUX_CHAN4    0x4
#define DMAMUX_CHAN5    0x5
#define DMAMUX_CHAN6    0x6
#define DMAMUX_CHAN7    0x7
#define DMAMUX_CHAN8    0x8
#define DMAMUX_CHAN9    0x9
#define DMAMUX_CHAN10   0xA
#define DMAMUX_CHAN11   0xB
#define DMAMUX_CHAN12   0xC
#define DMAMUX_CHAN13   0xD
#define DMAMUX_CHAN14   0xE
#define DMAMUX_CHAN15   0xF

#define DMAMUX_CHCFG_ADDR(a)      (DMAMUX_ADDR + (a))
#define DMAMUX_CHCFG_PTR(a)       ((volatile uint8_t*)DMAMUX_CHCFG_ADDR((a)))
#define DMAMUX_CHCFG(a)           (*(DMAMUX_CHCFG_PTR((a))))

#define DMAMUX_CHCFG_BB_ADDR(a,b) (BIT_2_BB2_ADDR( (DMAMUX_ADDR+(a)), (b)))
#define DMAMUX_CHCFG_BB_PTR(a,b)  ((volatile uint8_t*)DMAMUX_CHCFG_BB_ADDR((a),(b)))
#define DMAMUX_CHCFG_BB(a,b)      (*(DMAMUX_CHCFG_BB_PTR((a),(b))))

#if 0
#define DMAMUX_CHCFG0_ADDR          (DMAMUX_ADDR + DMAMUX_CHCFG0_OFFSET)
#define DMAMUX_CHCFG0_PTR           ((volatile uint8_t*)DMAMUX_CHCFG0_ADDR)
#define DMAMUX_CHCFG0               (*(DMAMUX_CHCFG0_PTR))
#define DMAMUX_CHCFG0_ENBL_BB_ADDR  (BIT_2_BB2_ADDR(DMAMUX_CHCFG0_ADDR, DMAMUX_ENBL_BIT))
#define DMAMUX_CHCFG0_ENBL_BB_PTR   ((volatile uint8_t*)DMAMUX_CHCFG0_ENBL_BB_ADDR)
#define DMAMUX_CHCFG0_ENBL_BB       (*(DMAMUX_CHCFG0_ENBL_BB_PTR))
#define DMAMUX_CHCFG0_TRIG_BB_ADDR  (BIT_2_BB2_ADDR(DMAMUX_CHCFG0_ADDR, DMAMUX_TRIG_BIT))
#define DMAMUX_CHCFG0_TRIG_BB_PTR   ((volatile uint8_t*)DMAMUX_CHCFG0_TRIG_BB_ADDR)
#define DMAMUX_CHCFG0_TRIG_BB       (*(DMAMUX_CHCFG0_TRIG_BB_PTR))

#define DMAMUX_CHCFG1_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG1_OFFSET)
#define DMAMUX_CHCFG1_PTR     ((volatile uint8_t*)DMAMUX_CHCFG1_ADDR)
#define DMAMUX_CHCFG1         (*(DMAMUX_CHCFG1_PTR))
#define DMAMUX_CHCFG2_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG2_OFFSET)
#define DMAMUX_CHCFG2_PTR     ((volatile uint8_t*)DMAMUX_CHCFG2_ADDR)
#define DMAMUX_CHCFG2         (*(DMAMUX_CHCFG2_PTR))
#define DMAMUX_CHCFG3_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG3_OFFSET)
#define DMAMUX_CHCFG3_PTR     ((volatile uint8_t*)DMAMUX_CHCFG3_ADDR)
#define DMAMUX_CHCFG3         (*(DMAMUX_CHCFG3_PTR))
#define DMAMUX_CHCFG4_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG4_OFFSET)
#define DMAMUX_CHCFG4_PTR     ((volatile uint8_t*)DMAMUX_CHCFG4_ADDR)
#define DMAMUX_CHCFG4         (*(DMAMUX_CHCFG4_PTR))
#define DMAMUX_CHCFG5_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG5_OFFSET)
#define DMAMUX_CHCFG5_PTR     ((volatile uint8_t*)DMAMUX_CHCFG5_ADDR)
#define DMAMUX_CHCFG5         (*(DMAMUX_CHCFG5_PTR))
#define DMAMUX_CHCFG6_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG6_OFFSET)
#define DMAMUX_CHCFG6_PTR     ((volatile uint8_t*)DMAMUX_CHCFG6_ADDR)
#define DMAMUX_CHCFG6         (*(DMAMUX_CHCFG6_PTR))
#define DMAMUX_CHCFG7_ADDR    (DMAMUX__ADDR + DMAMUX_CHCFG7_OFFSET)
#define DMAMUX_CHCFG7_PTR     ((volatile uint8_t*)DMAMUX_CHCFG7_ADDR)
#define DMAMUX_CHCFG7         (*(DMAMUX_CHCFG7_PTR))
#define DMAMUX_CHCFG8_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG8_OFFSET)
#define DMAMUX_CHCFG8_PTR     ((volatile uint8_t*)DMAMUX_CHCFG8_ADDR)
#define DMAMUX_CHCFG8         (*(DMAMUX_CHCFG8_PTR))
#define DMAMUX_CHCFG9_ADDR    (DMAMUX_ADDR + DMAMUX_CHCFG9_OFFSET)
#define DMAMUX_CHCFG9_PTR     ((volatile uint8_t*)DMAMUX_CHCFG9_ADDR)
#define DMAMUX_CHCFG9         (*(DMAMUX_CHCFG9_PTR))
#define DMAMUX_CHCFG10_ADDR   (DMAMUX_ADDR + DMAMUX_CHCFG10_OFFSET)
#define DMAMUX_CHCFG10_PTR    ((volatile uint8_t*)DMAMUX_CHCFG10_ADDR)
#define DMAMUX_CHCFG10        (*(DMAMUX_CHCFG10_PTR))
#define DMAMUX_CHCFG11_ADDR   (DMAMUX_ADDR + DMAMUX_CHCFG11_OFFSET)
#define DMAMUX_CHCFG11_PTR    ((volatile uint8_t*)DMAMUX_CHCFG11_ADDR)
#define DMAMUX_CHCFG11        (*(DMAMUX_CHCFG11_PTR))
#define DMAMUX_CHCFG12_ADDR   (DMAMUX_ADDR + DMAMUX_CHCFG12_OFFSET)
#define DMAMUX_CHCFG12_PTR    ((volatile uint8_t*)DMAMUX_CHCFG12_ADDR)
#define DMAMUX_CHCFG12        (*(DMAMUX_CHCFG12_PTR))
#define DMAMUX_CHCFG13_ADDR   (DMAMUX_ADDR + DMAMUX_CHCFG13_OFFSET)
#define DMAMUX_CHCFG13_PTR    ((volatile uint8_t*)DMAMUX_CHCFG13_ADDR)
#define DMAMUX_CHCFG13        (*(DMAMUX_CHCFG13_PTR))
#define DMAMUX_CHCFG14_ADDR   (DMAMUX_ADDR + DMAMUX_CHCFG14_OFFSET)
#define DMAMUX_CHCFG14_PTR    ((volatile uint8_t*)DMAMUX_CHCFG14_ADDR)
#define DMAMUX_CHCFG14        (*(DMAMUX_CHCFG14_PTR))
#define DMAMUX_CHCFG15_ADDR   (DMAMUX_ADDR + DMAMUX_CHCFG15_OFFSET)
#define DMAMUX_CHCFG15_PTR    ((volatile uint8_t*)DMAMUX_CHCFG15_ADDR)
#define DMAMUX_CHCFG15        (*(DMAMUX_CHCFG15_PTR))
#endif

/*******************************************************************************
* DMA
*******************************************************************************/
#define DMA_ADDR        0x40008000
#define DMA_CR_OFFSET   0x04
#define DMA_CERQ_OFFSET 0x1A
#define DMA_SERQ_OFFSET 0x1B
#define DMA_INT_OFFSET  0x24

#define DMA_CR_ADDR       (DMA_ADDR + DMA_CR_OFFSET)
#define DMA_CR_PTR        ((volatile uint32_t*)DMA_CR_ADDR)
#define DMA_CR            (*(DMA_CR_PTR))
#define DMA_ES_ADDR       (DMA_ADDR + DMA_ES_OFFSET)
#define DMA_ES_PTR        ((volatile uint32_t*)DMA_ES_ADDR)
#define DMA_ES            (*(DMA_ES_PTR))
#define DMA_ERQ_ADDR      (DMA_ADDR + DMA_ERQ_OFFSET)
#define DMA_ERQ_PTR       ((volatile uint32_t*)DMA_ERQ_ADDR)
#define DMA_ERQ           (*(DMA_ERQ_PTR))
#define DMA_EEI_ADDR      (DMA_ADDR + DMA_EEI_OFFSET)
#define DMA_EEI_PTR       ((volatile uint32_t*)DMA_EEI_ADDR)
#define DMA_EEI           (*(DMA_EEI_PTR))
#define DMA_CEEI_ADDR     (DMA_ADDR + DMA_CEEI_OFFSET)
#define DMA_CEEI_PTR      ((volatile uint8_t*)DMA_CEEI_ADDR)
#define DMA_CEEI          (*(DMA_CEEI_PTR))
#define DMA_SEEI_ADDR     (DMA_ADDR + DMA_SEEI_OFFSET)
#define DMA_SEEI_PTR      ((volatile uint8_t*)DMA_SEEI_ADDR)
#define DMA_SEEI          (*(DMA_SEEI_PTR))
#define DMA_CERQ_ADDR     (DMA_ADDR + DMA_CERQ_OFFSET)
#define DMA_CERQ_PTR      ((volatile uint8_t*)DMA_CERQ_ADDR)
#define DMA_CERQ          (*(DMA_CERQ_PTR))
#define DMA_SERQ_ADDR     (DMA_ADDR + DMA_SERQ_OFFSET)
#define DMA_SERQ_PTR      ((volatile uint8_t*)DMA_SERQ_ADDR)
#define DMA_SERQ          (*(DMA_SERQ_PTR))
#define DMA_CDNE_ADDR     (DMA_ADDR + DMA_CDNE_OFFSET)
#define DMA_CDNE_PTR      ((volatile uint8_t*)DMA_CDNE_ADDR)
#define DMA_CDNE          (*(DMA_CDNE_PTR))
#define DMA_SSRT_ADDR     (DMA_ADDR + DMA_SSRT_OFFSET)
#define DMA_SSRT_PTR      ((volatile uint8_t*)DMA_SSRT_ADDR)
#define DMA_SSRT          (*(DMA_SSRT_PTR))
#define DMA_CERR_ADDR     (DMA_ADDR + DMA_CERR_OFFSET)
#define DMA_CERR_PTR      ((volatile uint8_t*)DMA_CERR_ADDR)
#define DMA_CERR          (*(DMA_CERR_PTR))
#define DMA_CINT_ADDR     (DMA_ADDR + DMA_CINT_OFFSET)
#define DMA_CINT_PTR      ((volatile uint8_t*)DMA_CINT_ADDR)
#define DMA_CINT          (*(DMA_CINT_PTR))

#define DMA_INT_ADDR       (DMA_ADDR + DMA_INT_OFFSET)
#define DMA_INT_PTR        ((volatile uint32_t*)DMA_INT_ADDR)
#define DMA_INT            (*(DMA_INT_PTR))
#define DMA_INT_BB_ADDR(a) (BIT_2_BB2_ADDR(DMA_INT_ADDR,(a)))
#define DMA_INT_BB_PTR(a)  ((volatile uint32_t*)DMA_INT_BB_ADDR((a)))
#define DMA_INT_BB(a)      (*(DMA_INT_BB_PTR((a))))


#define DMA_ERR_ADDR      (DMA_ADDR + DMA_ERR_OFFSET)
#define DMA_ERR_PTR       ((volatile uint32_t*)DMA_ERR_ADDR)
#define DMA_ERR           (*(DMA_ERR_PTR))
#define DMA_HRS_ADDR      (DMA_ADDR + DMA_HRS_OFFSET)
#define DMA_HRS_PTR       ((volatile uint32_t*)DMA_HRS_ADDR)
#define DMA_HRS           (*(DMA_HRS_PTR))
#define DMA_DCHPRI3_ADDR  (DMA_ADDR + DMA_DCHPRI3_OFFSET)
#define DMA_DCHPRI3_PTR   ((volatile uint8_t*)DMA_DCHPRI3_ADDR)
#define DMA_DCHPRI3       (*(DMA_DCHPRI3_PTR))
#define DMA_DCHPRI2_ADDR  (DMA_ADDR + DMA_DCHPRI2_OFFSET)
#define DMA_DCHPRI2_PTR   ((volatile uint8_t*)DMA_DCHPRI2_ADDR)
#define DMA_DCHPRI2       (*(DMA_DCHPRI2_PTR))
#define DMA_DCHPRI1_ADDR  (DMA_ADDR + DMA_DCHPRI1_OFFSET)
#define DMA_DCHPRI1_PTR   ((volatile uint8_t*)DMA_DCHPRI1_ADDR)
#define DMA_DCHPRI1       (*(DMA_DCHPRI1_PTR))
#define DMA_DCHPRI0_ADDR  (DMA_ADDR + DMA_DCHPRI0_OFFSET)
#define DMA_DCHPRI0_PTR   ((volatile uint8_t*)DMA_DCHPRI0_ADDR)
#define DMA_DCHPRI0       (*(DMA_DCHPRI0_PTR))
#define DMA_DCHPRI7_ADDR  (DMA_ADDR + DMA_DCHPRI7_OFFSET)
#define DMA_DCHPRI7_PTR   ((volatile uint8_t*)DMA_DCHPRI7_ADDR)
#define DMA_DCHPRI7       (*(DMA_DCHPRI7_PTR))
#define DMA_DCHPRI6_ADDR  (DMA_ADDR + DMA_DCHPRI6_OFFSET)
#define DMA_DCHPRI6_PTR   ((volatile uint8_t*)DMA_DCHPRI6_ADDR)
#define DMA_DCHPRI6       (*(DMA_DCHPRI6_PTR))
#define DMA_DCHPRI5_ADDR  (DMA_ADDR + DMA_DCHPRI5_OFFSET)
#define DMA_DCHPRI5_PTR   ((volatile uint8_t*)DMA_DCHPRI5_ADDR)
#define DMA_DCHPRI5       (*(DMA_DCHPRI5_PTR))
#define DMA_DCHPRI4_ADDR  (DMA_ADDR + DMA_DCHPRI4_OFFSET)
#define DMA_DCHPRI4_PTR   ((volatile uint8_t*)DMA_DCHPRI4_ADDR)
#define DMA_DCHPRI4       (*(DMA_DCHPRI4_PTR))
#define DMA_DCHPRI11_ADDR (DMA_ADDR + DMA_DCHPRI11_OFFSET)
#define DMA_DCHPRI11_PTR  ((volatile uint8_t*)DMA_DCHPRI11_ADDR)
#define DMA_DCHPRI11      (*(DMA_DCHPRI11_PTR))
#define DMA_DCHPRI10_ADDR (DMA_ADDR + DMA_DCHPRI10_OFFSET)
#define DMA_DCHPRI10_PTR  ((volatile uint8_t*)DMA_DCHPRI10_ADDR)
#define DMA_DCHPRI10      (*(DMA_DCHPRI10_PTR))
#define DMA_DCHPRI9_ADDR  (DMA_ADDR + DMA_DCHPRI9_OFFSET)
#define DMA_DCHPRI9_PTR   ((volatile uint8_t*)DMA_DCHPRI9_ADDR)
#define DMA_DCHPRI9       (*(DMA_DCHPRI9_PTR))
#define DMA_DCHPRI8_ADDR  (DMA_ADDR + DMA_DCHPRI8_OFFSET)
#define DMA_DCHPRI8_PTR   ((volatile uint8_t*)DMA_DCHPRI8_ADDR)
#define DMA_DCHPRI8       (*(DMA_DCHPRI8_PTR))
#define DMA_DCHPRI15_ADDR (DMA_ADDR + DMA_DCHPRI15_OFFSET)
#define DMA_DCHPRI15_PTR  ((volatile uint8_t*)DMA_DCHPRI15_ADDR)
#define DMA_DCHPRI15      (*(DMA_DCHPRI15_PTR))
#define DMA_DCHPRI14_ADDR (DMA_ADDR + DMA_DCHPRI14_OFFSET)
#define DMA_DCHPRI14_PTR  ((volatile uint8_t*)DMA_DCHPRI14_ADDR)
#define DMA_DCHPRI14      (*(DMA_DCHPRI14_PTR))
#define DMA_DCHPRI13_ADDR (DMA_ADDR + DMA_DCHPRI13_OFFSET)
#define DMA_DCHPRI13_PTR  ((volatile uint8_t*)DMA_DCHPRI13_ADDR)
#define DMA_DCHPRI13      (*(DMA_DCHPRI13_PTR))
#define DMA_DCHPRI12_ADDR (DMA_ADDR + DMA_DCHPRI12_OFFSET)
#define DMA_DCHPRI12_PTR  ((volatile uint8_t*)DMA_DCHPRI12_ADDR)
#define DMA_DCHPRI12      (*(DMA_DCHPRI12_PTR))

#define DMA_CHAN0     0U
#define DMA_CHAN1     1U
#define DMA_CHAN2     2U
#define DMA_CHAN3     3U
#define DMA_CHAN4     4U
#define DMA_CHAN5     5U
#define DMA_CHAN6     6U
#define DMA_CHAN7     7U
#define DMA_CHAN8     8U
#define DMA_CHAN9     9U
#define DMA_CHAN10    10U
#define DMA_CHAN11    11U
#define DMA_CHAN12    12U
#define DMA_CHAN13    13U
#define DMA_CHAN14    14U
#define DMA_CHAN15    15U

#define DMA_SADDR_OFFSET    (DMA_ADDR + 0x1000)
#define DMA_SOFF_OFFSET     (DMA_ADDR + 0x1004)
#define DMA_ATTR_OFFSET     (DMA_ADDR + 0x1006)
#define DMA_NBYTES_OFFSET   (DMA_ADDR + 0x1008)
#define DMA_SLAST_OFFSET    (DMA_ADDR + 0x100C)
#define DMA_DADDR_OFFSET    (DMA_ADDR + 0x1010)
#define DMA_DOFF_OFFSET     (DMA_ADDR + 0x1014)
#define DMA_CITER_OFFSET    (DMA_ADDR + 0x1016)
#define DMA_DLASTSGA_OFFSET (DMA_ADDR + 0x1018)
#define DMA_CSR_OFFSET      (DMA_ADDR + 0x101C)
#define DMA_BITER_OFFSET    (DMA_ADDR + 0x101E)

#define DMA_SADDR_ADDR(a)    (((a)*32U) + DMA_SADDR_OFFSET)
#define DMA_SADDR_PTR(a)     ((volatile uint32_t*)DMA_SADDR_ADDR((a)))
#define DMA_SADDR(a)         (*(DMA_SADDR_PTR((a))))

#define DMA_SOFF_ADDR(a)     (((a)*32U) + DMA_SOFF_OFFSET)
#define DMA_SOFF_PTR(a)      ((volatile uint16_t*)DMA_SOFF_ADDR((a)))
#define DMA_SOFF(a)          (*(DMA_SOFF_PTR((a))))

#define DMA_ATTR_ADDR(a)     (((a)*32U) + DMA_ATTR_OFFSET)
#define DMA_ATTR_PTR(a)      ((volatile uint16_t*)DMA_ATTR_ADDR((a)))
#define DMA_ATTR(a)          (*(DMA_ATTR_PTR((a))))

#define DMA_NBYTES_ADDR(a)     (((a)*32U) + DMA_NBYTES_OFFSET)
#define DMA_NBYTES_PTR(a)      ((volatile uint32_t*)DMA_NBYTES_ADDR((a)))
#define DMA_NBYTES(a)          (*(DMA_NBYTES_PTR((a))))

#define DMA_SLAST_ADDR(a)    (((a)*32U) + DMA_SLAST_OFFSET)
#define DMA_SLAST_PTR(a)     ((volatile uint32_t*)DMA_SLAST_ADDR((a)))
#define DMA_SLAST(a)         (*(DMA_SLAST_PTR((a))))

#define DMA_DADDR_ADDR(a)    (((a)*32U) + DMA_DADDR_OFFSET)
#define DMA_DADDR_PTR(a)     ((volatile uint32_t*)DMA_DADDR_ADDR((a)))
#define DMA_DADDR(a)         (*(DMA_DADDR_PTR((a))))

#define DMA_DOFF_ADDR(a)     (((a)*32U) + DMA_DOFF_OFFSET)
#define DMA_DOFF_PTR(a)      ((volatile uint16_t*)DMA_DOFF_ADDR((a)))
#define DMA_DOFF(a)          (*(DMA_DOFF_PTR((a))))

#define DMA_CITER_ADDR(a)  (((a)*32U) + DMA_CITER_OFFSET)
#define DMA_CITER_PTR(a)   ((volatile uint16_t*)DMA_CITER_ADDR((a)))
#define DMA_CITER(a)       (*(DMA_CITER_PTR((a))))

#define DMA_DLASTSGA_ADDR(a) (((a)*32U) + DMA_DLASTSGA_OFFSET)
#define DMA_DLASTSGA_PTR(a)  ((volatile uint32_t*)DMA_DLASTSGA_ADDR((a)))
#define DMA_DLASTSGA(a)      (*(DMA_DLASTSGA_PTR((a))))

#define DMA_CSR_DONE_BIT        7U
#define DMA_CSR_ACTIVE_BIT      6U
#define DMA_CSR_DREQ_BIT        3U
#define DMA_CSR_INTMAJ_BIT      1U

#define DMA_CSR_ADDR(a)      (((a)*32U) + DMA_CSR_OFFSET)
#define DMA_CSR_PTR(a)       ((volatile uint16_t*)DMA_CSR_ADDR((a)))
#define DMA_CSR(a)           (*(DMA_CSR_PTR((a))))

#define DMA_CSR_BB_ADDR(a,b)  (BIT_2_BB2_ADDR(DMA_CSR_ADDR(a),(b)))
#define DMA_CSR_BB_PTR(a,b)   ((volatile uint32_t*)DMA_CSR_BB_ADDR((a),(b)))
#define DMA_CSR_BB(a,b)       (*(DMA_CSR_BB_PTR((a),(b))))

#define DMA_BITER_ADDR(a) (((a)*32U) + DMA_BITER_OFFSET)
#define DMA_BITER_PTR(a)  ((volatile uint16_t*)DMA_BITER_ADDR((a)))
#define DMA_BITER(a)      (*(DMA_BITER_PTR((a))))

#endif
