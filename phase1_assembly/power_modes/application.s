/*
 * application.s
 *
 * Shaun Weise
 * June 7 2012
 */

    .syntax unified
    .thumb

    .text
    .align 2

    .global asm_main
    .global main_loop
    .thumb_func

/* Main Loop */
asm_main:                                         /* Called by our start code */

    /*
     * Push the link register on the stack so we can remember where we
     * came from (turns out heritage _is_ important ... who knew??).
     */
    push { lr }

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

    /*****************************************/
    /* Definitions                           */
    /*****************************************/

    .set PORTA_CTRL_BASE, 0x40049000
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

    .set PORTE_CTRL_BASE, 0x4004d000
    .set PIN_26_OFFSET, (4 * 26)

    .set SW_0_CTRL_ADDR,       PORTE_CTRL_BASE + PIN_26_OFFSET

    .set GPIO_ENABLE, (0x001 << 8)
    .set PULL_UP_ENABLE, (1 << 1)
    .set PULL_UP_SELECT, (1 << 0)
    .set PORT_CTRL_FLAGS, (GPIO_ENABLE | PULL_UP_ENABLE | PULL_UP_SELECT)

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
    .set PORTE_INPUT_REG, PORTE_BASE_ADDR + 0x10

    .set SW_0_MASK, (1 << 26)

    /*
     * SIM Defines
     */

    .set SIM_SCGC5, 0x40048038
    .set PORTA_ENABLE, (1 << 9)
    .set PORTC_ENABLE, (1 << 11)
    .set PORTE_ENABLE, (1 << 13)
    .set SIM_SCGC5_FLAGS, (PORTA_ENABLE | PORTE_ENABLE | PORTC_ENABLE)


    /* My stuff */
    .set PORTC_PCR4_ADDR,       0x4004B010
    .set PORTC_PCR4_CFG,        0x000A0103 /* Config, gpio, pull up */
    .set SIM_SCGC4_ADDR,        0x40048034
    .set SIM_SCGC4_LLWU_ENBL_MASK, 0x10000000
    .set LLWU_PE3_ADDR,         0x4007C002 /* Register controlling LLWU_P8, PTC4 */
    .set LLWU_PE3_CFG,          0x02       /* Falling edge detect in low leakage wake-up unit */
    .set MCG_C6,                0x40064005
    .set MCG_C6_CME0_MASK,      0x20
    .set MC_PMPROT_ADDR,        0x4007E002
    .set MC_PMPROT_ALLS_MASK,   0x10          /* Allow LLS Mode */
    .set MC_PMCTRL_ADDR,        0x4007E003
    .set MC_PMCTRL_LPLLSM_MASK, 0x03          /* Set LLS low-power mode */
    .set SCR,                   0xE000ED10
    .set SCR_SLEEPDEEP_MASK,    0x00000004

    /*****************************************/
    /* CODE                                  */
    /*****************************************/

    nop
    /* Enable GPIO PORTS (Clock Gate) */
    ldr r0, =SIM_SCGC5_FLAGS
    ldr r1, =SIM_SCGC5
    str r0, [r1]

    /* Configure PCRs */
    ldr r0, =PORT_CTRL_FLAGS
    ldr r1, =LED_ORANGE_CTRL_ADDR
    ldr r2, =LED_YELLOW_CTRL_ADDR
    ldr r3, =LED_GREEN_CTRL_ADDR
    ldr r4, =LED_BLUE_CTRL_ADDR
    ldr r5, =SW_1_CTRL_ADDR
    ldr r6, =SW_0_CTRL_ADDR
    str r0, [r1]
    str r0, [r2]
    str r0, [r3]
    str r0, [r4]
    str r0, [r5]
    str r0, [r6]

    /* Configure GPIOs
     * Note: 0 = input, 1 = ouput (all pins default hi z inputs).*/
    ldr r0, =LEDS_MASK                                      /* LEDs = Outputs */
    ldr r1, =PORTA_DATA_DIRECTION_ADDR
    str r0, [r1]

    /*******************/
    /* Setup power LLS */
    /*******************/

    /* Configure PTC4(LLWU_P8): gpio, falling edge int, pullup enble */
    ldr r0, =PORTC_PCR4_CFG
    ldr r1, =PORTC_PCR4_ADDR
    str r0, [r1]

    /* Enable the LLWU */
    ldr r0, =SIM_SCGC4_LLWU_ENBL_MASK
    ldr r1, =SIM_SCGC4_ADDR
    str r0, [r1]

    /* Configure PTC4(LLWU_P8) as a wakeup source with falling edge detect in
       the low leakage wake up unit (LLWU)  */
    ldr r0, =LLWU_PE3_CFG
    ldr r1, =LLWU_PE3_ADDR
    strb r0, [r1]

    /* Disable the clock monitor */
    ldr r0, =MCG_C6_CME0_MASK
    ldr r1, =MCG_C6
    bic r1,r1,r0

main_loop:

leds_all_on:
    ldr r0, =LEDS_MASK
    ldr r1, =PORTA_CLR_REG
    str r0, [r1]

    ldr r9, =0xcffff
    bl delay

leds_all_off:
    ldr r0, =LEDS_MASK
    ldr r1, =PORTA_SET_REG
    str r0, [r1]

    ldr r9, =0xcffff
    bl delay

    /* Test switch */
    ldr r0, =PORTA_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_1_MASK
    ands r1, r2
    bne main_loop

enter_lls_power_mode:

    /* Allow LLS power mode  */
    ldr r0, =MC_PMPROT_ALLS_MASK
    ldr r1, =MC_PMPROT_ADDR
    strb r0, [r1]

    /* Set mcu into LLS power mode */
    ldr r0, =MC_PMCTRL_LPLLSM_MASK
    ldr r1, =MC_PMCTRL_ADDR
    strb r0, [r1]

    /* Sleep in SLEEPDEEP mode */
    nop
    ldr r0, =SCR_SLEEPDEEP_MASK
    ldr r1, =SCR
    str r0, [r1]

    /* Finally enter LLS! But never come back */
    nop
    wfi

    b main_loop

    /*
     * If main_loop didn't loop forever we'd want to
     * pull the stacked link register into the program counter so we
     * can return to the start code for epilogue processing (if applicable).
     */
    pop { pc }

/*****************************************************************************
 * delay
 ****************************************************************************/
    .global delay
    .align 2
    .thumb_func
    .type  sleep, %function
delay:
    push { r0 }
    mov r0, r9
delay_loop:
    subs r0, #1
    bne delay_loop
    pop { r0 }
    bx lr

    .end
