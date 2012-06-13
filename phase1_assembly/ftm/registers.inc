/*
 * registers.inc
 *
 * Register definitions for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * By S. Holford
 * Based on work by Rob Laswick and others
 * May 27 2012
 */

/*
 * Tower Schematic:
 *
 *   LED0: PTA11 (active low) (orange)
 *   LED1: PTA28 (active low) (yellow)
 *   LED2: PTA29 (active low) (green)
 *   LED3: PTA10 (active low) (blue)
 *   SW0 : PTE26 (active low) (needs internal pull up)
 *   SW1 : PTA19 (active low) (needs internal pull up)
 */

/*
 * PORT Controller Defines
 */

    .set PORTA_CTRL_BASE, 0x40049000
    .set PIN_6_OFFSET, (4 * 6)
    .set PIN_11_OFFSET, (4 * 11)
    .set PIN_28_OFFSET, (4 * 28)
    .set PIN_29_OFFSET, (4 * 29)
    .set PIN_10_OFFSET, (4 * 10)
    .set PIN_19_OFFSET, (4 * 19)

    .set LED_ORANGE_CTRL_ADDR, PORTA_CTRL_BASE + PIN_11_OFFSET
    .set LED_YELLOW_CTRL_ADDR, PORTA_CTRL_BASE + PIN_28_OFFSET
    .set LED_GREEN_CTRL_ADDR,  PORTA_CTRL_BASE + PIN_29_OFFSET
    .set LED_BLUE_CTRL_ADDR,   PORTA_CTRL_BASE + PIN_10_OFFSET

    .set SW_1_CTRL_ADDR,       PORTA_CTRL_BASE + PIN_19_OFFSET
    .set FTM0_CH3_CTRL_ADDR, PORTA_CTRL_BASE + PIN_6_OFFSET

    .set PORTC_CTRL_BASE, 0x4004B000
    .set PIN_16_OFFSET, (4 * 16)
    .set PIN_17_OFFSET, (4 * 17)

    .set PORTE_CTRL_BASE, 0x4004D000
    .set PIN_8_OFFSET, (4 * 8)
    .set PIN_9_OFFSET, (4 * 9)
    .set PIN_26_OFFSET, (4 * 26)

    .set UART3_RX_CTRL_ADDR,   PORTC_CTRL_BASE + PIN_16_OFFSET
    .set UART3_TX_CTRL_ADDR,   PORTC_CTRL_BASE + PIN_17_OFFSET

    .set UART5_RX_CTRL_ADDR,   PORTE_CTRL_BASE + PIN_8_OFFSET
    .set UART5_TX_CTRL_ADDR,   PORTE_CTRL_BASE + PIN_9_OFFSET
    .set SW_0_CTRL_ADDR,       PORTE_CTRL_BASE + PIN_26_OFFSET

    .set GPIO_ENABLE, (0x001 << 8)
    .set ALT3_ENABLE, (0b011 << 8)
    .set PULL_UP_ENABLE, (1 << 1)
    .set PULL_UP_SELECT, (1 << 0)
    .set SLOW_SLEW_SELECT, (1 << 2)
    .set HIGH_DRIVE_SELECT, (1 << 6)
    .set PORT_CTRL_FLAGS, (GPIO_ENABLE | PULL_UP_ENABLE | PULL_UP_SELECT)
    .set PORTC_UART_CTRL_FLAGS, (ALT3_ENABLE)
    .set PORTE_UART_CTRL_FLAGS, (ALT3_ENABLE)
    .set PORTA_FTM0_CH3_CTRL_FLAGS, (ALT3_ENABLE)

/*
 * GPIO Controller Defines
 */

    .set PORTA_BASE_ADDR, 0x400ff000
    .set PORTA_OUTPUT_REG,          PORTA_BASE_ADDR + 0x0
    .set PORTA_SET_REG,             PORTA_BASE_ADDR + 0x4
    .set PORTA_CLR_REG,             PORTA_BASE_ADDR + 0x8
    .set PORTA_TGL_REG,             PORTA_BASE_ADDR + 0xc
    .set PORTA_INPUT_REG,           PORTA_BASE_ADDR + 0x10
    .set PORTA_DATA_DIRECTION_ADDR, PORTA_BASE_ADDR + 0x14

    .set LED_ORANGE, (1 << 11)
    .set LED_YELLOW, (1 << 28)
    .set LED_GREEN,  (1 << 29)
    .set LED_BLUE,   (1 << 10)
    .set LEDS_MASK,  (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE)

    .set SW_1_MASK, (1 << 19)

    .set PORTE_BASE_ADDR, 0x400ff100
    .set PORTE_INPUT_REG,           PORTE_BASE_ADDR + 0x10

    .set SW_0_MASK, (1 << 26)

/*
 * SIM Defines
 */

    .set SIM_SCGC1, 0x40048028
    .set UART5_ENABLE, (1 << 11)

    .set SIM_SCGC3, 0x40048030
    .set ADC1_ENABLE, (1 << 27)

    .set SIM_SCGC4, 0x40048034
    .set UART3_ENABLE, (1 << 13)

    .set SIM_SCGC5, 0x40048038
    .set PORTA_ENABLE, (1 << 9)
    .set PORTC_ENABLE, (1 << 11)
    .set PORTE_ENABLE, (1 << 13)

    .set SIM_SCGC6, 0x4004803C
    .set DMAMUX0_ENABLE, (1 << 1)
    .set DMAMUX1_ENABLE, (1 << 2)
    .set PIT_ENABLE, (1 << 23)
    .set FTM0_ENABLE, (1 << 24)


    .set SIM_SCGC7, 0x40048040
    .set EDMA_ENABLE, (1 << 1)

/*
 * UART 3 defines
 */

    .set UART3_BASE_PTR, 0x4006D000
    .set UART3_BDH, UART3_BASE_PTR+0
    .set UART3_BDL, UART3_BASE_PTR+1
    .set UART3_C1, UART3_BASE_PTR+2
    .set UART3_C2, UART3_BASE_PTR+3
    .set UART3_S1, UART3_BASE_PTR+4
    .set UART3_S2, UART3_BASE_PTR+5
    .set UART3_C3, UART3_BASE_PTR+6
    .set UART3_D, UART3_BASE_PTR+7
    .set UART3_MA1, UART3_BASE_PTR+8
    .set UART3_MA2, UART3_BASE_PTR+9
    .set UART3_C4, UART3_BASE_PTR+10
    .set UART3_C5, UART3_BASE_PTR+11
    
/*
 * UART 5 defines
 */

    .set UART5_BASE_PTR, 0x400EB000
    .set UART5_BDH, UART5_BASE_PTR+0
    .set UART5_BDL, UART5_BASE_PTR+1
    .set UART5_C1, UART5_BASE_PTR+2
    .set UART5_C2, UART5_BASE_PTR+3
    .set UART5_S1, UART5_BASE_PTR+4
    .set UART5_S2, UART5_BASE_PTR+5
    .set UART5_C3, UART5_BASE_PTR+6
    .set UART5_D, UART5_BASE_PTR+7
    .set UART5_MA1, UART5_BASE_PTR+8
    .set UART5_MA2, UART5_BASE_PTR+9
    .set UART5_C4, UART5_BASE_PTR+10
    .set UART5_C5, UART5_BASE_PTR+11
    
    .set UART_S1_TDRE, (1 << 7)

/* PIT Control */

    .set PIT_CTRL_BASE, 0x40037000

    .set PIT_MCR, (PIT_CTRL_BASE)
    .set PIT2_LDVAL, (PIT_CTRL_BASE + 0x120)
    .set PIT2_CVAL, (PIT_CTRL_BASE + 0x124)
    .set PIT2_TCTRL, (PIT_CTRL_BASE + 0x128)
    .set PIT2_TFLG, (PIT_CTRL_BASE + 0x12C)

    .set MCR_MDIS, (1 << 1)
    .set MCR_FRZ, (1 << 0)
    .set TCTRL_TIE, (1 << 1)
    .set TCTRL_TEN, (1 << 0)
    .set TFLG_TIF, (1 << 0)

/* FTM Control */

    .set FTM0_CTRL_BASE, 0x40038000

    .set FTM0_SC,(FTM0_CTRL_BASE)
    .set FTM0_CNT,(FTM0_CTRL_BASE + 0x4)
    .set FTM0_MOD,(FTM0_CTRL_BASE + 0x8)

    .set FTM0_C0SC,(FTM0_CTRL_BASE + 0xC)
    .set FTM0_C0V,(FTM0_CTRL_BASE + 0x10)
    .set FTM0_C1SC,(FTM0_CTRL_BASE + 0x14)
    .set FTM0_C1V,(FTM0_CTRL_BASE + 0x18)
    .set FTM0_C2SC,(FTM0_CTRL_BASE + 0x1C)
    .set FTM0_C2V,(FTM0_CTRL_BASE + 0x20)
    .set FTM0_C3SC,(FTM0_CTRL_BASE + 0x24)
    .set FTM0_C3V,(FTM0_CTRL_BASE + 0x28)
    .set FTM0_C4SC,(FTM0_CTRL_BASE + 0x2C)
    .set FTM0_C4V,(FTM0_CTRL_BASE + 0x30)
    .set FTM0_C5SC,(FTM0_CTRL_BASE + 0x34)
    .set FTM0_C5V,(FTM0_CTRL_BASE + 0x38)
    .set FTM0_C6SC,(FTM0_CTRL_BASE + 0x3C)
    .set FTM0_C6V,(FTM0_CTRL_BASE + 0x40)
    .set FTM0_C7SC,(FTM0_CTRL_BASE + 0x44)
    .set FTM0_C7V,(FTM0_CTRL_BASE + 0x48)

    .set FTM0_CNTIN,(FTM0_CTRL_BASE + 0x4C)
    .set FTM0_STAT,(FTM0_CTRL_BASE + 0x50)
    .set FTM0_MODE,(FTM0_CTRL_BASE + 0x54)
    .set FTM0_SYNC,(FTM0_CTRL_BASE + 0x58)
    .set FTM0_OUTINIT,(FTM0_CTRL_BASE + 0x5C)
    .set FTM0_OUTMASK,(FTM0_CTRL_BASE + 0x60)
    .set FTM0_COMBINE,(FTM0_CTRL_BASE + 0x64)
    .set FTM0_DEADTIME,(FTM0_CTRL_BASE + 0x68)
    .set FTM0_EXTTRIG,(FTM0_CTRL_BASE + 0x6C)
    .set FTM0_POL,(FTM0_CTRL_BASE + 0x70)
    .set FTM0_FMS,(FTM0_CTRL_BASE + 0x74)
    .set FTM0_FILTER,(FTM0_CTRL_BASE + 0x78)
    .set FTM0_FLTCTRL,(FTM0_CTRL_BASE + 0x7C)
    .set FTM0_QDCTRL,(FTM0_CTRL_BASE + 0x80)
    .set FTM0_CONF,(FTM0_CTRL_BASE + 0x84)
    .set FTM0_FLTPOL,(FTM0_CTRL_BASE + 0x88)
    .set FTM0_SYNCONF,(FTM0_CTRL_BASE + 0x8C)
    .set FTM0_INVCTRL,(FTM0_CTRL_BASE + 0x90)
    .set FTM0_SWOCTRL,(FTM0_CTRL_BASE + 0x94)
    .set FTM0_PWMLOAD,(FTM0_CTRL_BASE + 0x98)

    .set FTM_SC_SYSCLK, (0b001 << 3)
    .set FTM_SC_PRE1, (0b0000 << 0)
    .set FTM_SC_PRE8, (0b0011 << 0)

    .set FTM_CH_MSB, (1 << 5)
    .set FTM_CH_MSA, (1 << 4)
    .set FTM_CH_ELSB, (1 << 3)
    .set FTM_CH_ELSA, (1 << 2)

    .set FTM_MODE_WPDIS, (1 << 2)
    .set FTM_MODE_FTMEN, (1 << 0)
    .set FTM_SYNC_CNTMAX, (1 << 1)


    .set FTM_SYNCONF_MODE, (1 << 7)
    .set FTM_PWMLOAD_EN, (1 << 9)


/* 
 * DMA MUX Defines
 */
    .set DMAMUX0_CHCFG_BASE, 0x40021000
    .set DMAMUX1_CHCFG_BASE, 0x40022000

    .set DMAMUX_ENABLE, (1 << 7)
    .set DMAMUX_TRIGGER, (1 << 6)
    .set DMAMUX_SOURCE_MASK, 0x3F

    .set DMAMUX_SRC_DISABLE, 0
    .set DMAMUX_SRC_UART0_RX, 2
    .set DMAMUX_SRC_UART0_TX, 3
    .set DMAMUX_SRC_UART1_RX, 4
    .set DMAMUX_SRC_UART1_TX, 5
    .set DMAMUX_SRC_UART2_RX, 6
    .set DMAMUX_SRC_UART2_TX, 7
    .set DMAMUX_SRC_UART3_RX, 8
    .set DMAMUX_SRC_UART3_TX, 9
    .set DMAMUX_SRC_UART4_RX, 10
    .set DMAMUX_SRC_UART4_TX, 11
    .set DMAMUX_SRC_UART5_RX, 12
    .set DMAMUX_SRC_UART5_TX, 13

    .set DMAMUX_SRC_I2S0_RX, 14
    .set DMAMUX_SRC_I2S0_TX, 15
    .set DMAMUX_SRC_SPI0_RX, 16
    .set DMAMUX_SRC_SPI0_TX, 17
    .set DMAMUX_SRC_SPI1_RX, 18
    .set DMAMUX_SRC_SPI1_TX, 19
    .set DMAMUX_SRC_I2C0, 22
    .set DMAMUX_SRC_I2C1, 23

    .set DMAMUX_SRC_FTM0_0, 24
    .set DMAMUX_SRC_FTM0_1, 25
    .set DMAMUX_SRC_FTM0_2, 26
    .set DMAMUX_SRC_FTM0_3, 27
    .set DMAMUX_SRC_FTM0_4, 28
    .set DMAMUX_SRC_FTM0_5, 29
    .set DMAMUX_SRC_FTM0_6, 30
    .set DMAMUX_SRC_FTM0_7, 31
    .set DMAMUX_SRC_FTM1_0, 32
    .set DMAMUX_SRC_FTM1_1, 33
    .set DMAMUX_SRC_FTM2_0, 34
    .set DMAMUX_SRC_FTM2_1, 35
    .set DMAMUX_SRC_FTM3_0, 36
    .set DMAMUX_SRC_FTM3_1, 37
    .set DMAMUX_SRC_FTM3_2, 38
    .set DMAMUX_SRC_FTM3_3, 39

    .set DMAMUX_SRC_ADC0, 40
    .set DMAMUX_SRC_ADC1, 41
    .set DMAMUX_SRC_CMP0, 42
    .set DMAMUX_SRC_CMP1, 43
    .set DMAMUX_SRC_CMP2, 44
    .set DMAMUX_SRC_DAC0, 45
    .set DMAMUX_SRC_DAC1, 46
    .set DMAMUX_SRC_CMT, 47
    .set DMAMUX_SRC_PDB, 48
    .set DMAMUX_SRC_PORTA, 49
    .set DMAMUX_SRC_PORTB, 50
    .set DMAMUX_SRC_PORTC, 51
    .set DMAMUX_SRC_PORTD, 52
    .set DMAMUX_SRC_PORTE, 53

    .set DMAMUX_SRC_FTM3_4, 54
    .set DMAMUX_SRC_FTM3_5, 55
    .set DMAMUX_SRC_FTM3_6, 56
    .set DMAMUX_SRC_FTM3_7, 57

    .set DMAMUX_SRC_DMA0_ON, 58
    .set DMAMUX_SRC_DMA1_ON, 59
    .set DMAMUX_SRC_DMA2_ON, 60
    .set DMAMUX_SRC_DMA3_ON, 61
    .set DMAMUX_SRC_DMA4_ON, 62
    .set DMAMUX_SRC_DMA5_ON, 63

/*
 * eDMA Defines
 */
    .set DMA_CR,   0x40008000
    .set DMA_ES,   0x40008004
    .set DMA_ERQ,  0x4000800C
    .set DMA_EEI,  0x40008014
    .set DMA_CEII, 0x40008018
    .set DMA_SEII, 0x40008019
    .set DMA_CERQ, 0x4000801A
    .set DMA_SERQ, 0x4000801B
    .set DMA_CDNE, 0x4000801C
    .set DMA_SSRT, 0x4000801D
    .set DMA_CERR, 0x4000801E
    .set DMA_CINT, 0x4000801F
    .set DMA_INT,  0x40008024
    .set DMA_ERR,  0x4000802C
    .set DMA_HRS,  0x40008034
    
    .set DMA_DCHPRI3, 0x40008100
    .set DMA_DCHPRI2, 0x40008101
    .set DMA_DCHPRI1, 0x40008102
    .set DMA_DCHPRI0, 0x40008103

    .set DMA_DCHPRI7, 0x40008104
    .set DMA_DCHPRI6, 0x40008105
    .set DMA_DCHPRI5, 0x40008106
    .set DMA_DCHPRI4, 0x40008107

    .set DMA_DCHPRI11, 0x40008108
    .set DMA_DCHPRI10, 0x40008109
    .set DMA_DCHPRI9,  0x4000810A
    .set DMA_DCHPRI8,  0x4000810B

    .set DMA_DCHPRI15, 0x4000810C
    .set DMA_DCHPRI14, 0x4000810D
    .set DMA_DCHPRI13, 0x4000810E
    .set DMA_DCHPRI12, 0x4000810F

    
/*
 * Base address and offset / aliases for each
 * TCD. There are 16 TCD's one for each DMA channel.
 * The TCD's are all 32 byte long, indexed by the
 * offsets below.
 */

    .set DMA_TCD_BASE, 0x40009000
    .set DMA_TCD0, (DMA_TCD_BASE)
    .set DMA_TCD1, (DMA_TCD0+32)
    .set DMA_TCD2, (DMA_TCD1+32)
    .set DMA_TCD3, (DMA_TCD2+32)
    .set DMA_TCD4, (DMA_TCD3+32)
    .set DMA_TCD5, (DMA_TCD4+32)
    .set DMA_TCD6, (DMA_TCD5+32)
    .set DMA_TCD7, (DMA_TCD6+32)
    .set DMA_TCD8, (DMA_TCD7+32)
    .set DMA_TCD9, (DMA_TCD8+32)
    .set DMA_TCD10, (DMA_TCD9+32)
    .set DMA_TCD11, (DMA_TCD10+32)
    .set DMA_TCD12, (DMA_TCD11+32)
    .set DMA_TCD13, (DMA_TCD12+32)
    .set DMA_TCD14, (DMA_TCD13+32)
    .set DMA_TCD15, (DMA_TCD14+32)

    .set DMA_TCD_SADDR, 0
    .set DMA_TCD_SOFF, 4
    .set DMA_TCD_ATTR, 6
    .set DMA_TCD_NBYTES, 8
    .set DMA_TCD_SLAST, 12
    .set DMA_TCD_DADDR, 16
    .set DMA_TCD_DOFF, 20
    .set DMA_TCD_CITER, 22
    .set DMA_TCD_DLASTSGA, 24
    .set DMA_TCD_CSR, 28
    .set DMA_TCD_BITER, 30

/*
 * ADC Defines
 */

    .set ADC0_SC1A, 0x4003B000
    .set ADC0_SC1B, 0x4003B004
    .set ADC0_CFG1, 0x4003B008
    .set ADC0_CFG2, 0x4003B00C
    .set ADC0_RA, 0x4003B010
    .set ADC0_RB, 0x4003B014
    .set ADC0_CV1, 0x4003B018
    .set ADC0_CV2, 0x4003B01C
    .set ADC0_SC1, 0x4003B020
    .set ADC0_SC3, 0x4003B024
    .set ADC0_OFS, 0x4003B028   
    .set ADC0_PG, 0x4003B02C 
    .set ADC0_MG, 0x4003B030
    .set ADC0_CLPD, 0x4003B034
    .set ADC0_CLPS, 0x4003B038
    .set ADC0_CLP4, 0x4003B03C
    .set ADC0_CLP3, 0x4003B040
    .set ADC0_CLP2, 0x4003B044
    .set ADC0_CLP1, 0x4003B048 
    .set ADC0_CLP0, 0x4003B04C
    .set ADC0_PGA, 0x4003B050
    .set ADC0_CLMD, 0x4003B054 
    .set ADC0_CLMS, 0x4003B058 
    .set ADC0_CLM4, 0x4003B05c
    .set ADC0_CLM3, 0x4003B060
    .set ADC0_CLM2, 0x4003B064
    .set ADC0_CLM1, 0x4003B068
    .set ADC0_CLM0, 0x4003B06C
/* ADC1 */
    .set ADC1_SC1A, 0x400BB000
    .set ADC1_SC1B, 0x400BB004
    .set ADC1_CFG1, 0x400BB008
    .set ADC1_CFG2, 0x400BB00C 
    .set ADC1_RA, 0x400BB010 
    .set ADC1_RB, 0x400BB014 
    .set ADC1_CV1, 0x400BB018 
    .set ADC1_CV2, 0x400BB01C 
    .set ADC1_SC2, 0x400BB020 
    .set ADC1_SC3, 0x400BB024 
    .set ADC1_OFS, 0x400BB028 
    .set ADC1_PG, 0x400BB02C 
    .set ADC1_MG, 0x400BB030 
    .set ADC1_CLPD, 0x400BB034 
    .set ADC1_CLPS, 0x400BB038 
    .set ADC1_CLP4, 0x400BB03C 
    .set ADC1_CLP3, 0x400BB040 
    .set ADC1_CLP2, 0x400BB044 
    .set ADC1_CLP1, 0x400BB048 
    .set ADC1_CLP0, 0x400BB04C 
    .set ADC1_CLMD, 0x400BB054 
    .set ADC1_CLMS, 0x400BB058 
    .set ADC1_CLM4, 0x400BB05C 
    .set ADC1_CLM3, 0x400BB060 
    .set ADC1_CLM2, 0x400BB064 
    .set ADC1_CLM1, 0x400BB068 
    .set ADC1_CLM0, 0x400BB06C 

    .set ADC1_CFG1_BITS, ((1 << 7) | (1 << 4) | (0b001 << 2))  /* Low Power, Long Sample Time, 12 bits */

