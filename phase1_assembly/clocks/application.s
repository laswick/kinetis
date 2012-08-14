/*
 * application.s
 *
 * Example user/application code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Clock setup Demonstration:
 *  -Set TWR_SER CCA J3 1-2 to route 25MHz into CLKIN0
 *  -Set TWR-K60N512 CCA J6 2-3 to route CLKIN0 int EXTAL
 *  -Applicatino tries to setup PLL on 25MHz external clock to 100MHz core.
 *  -If it fails to lock (e.g. don't have the TWR-SER connected, fail safe to
 *   internal clock
 *
 * James McAnanama
 * May 11 2012
 */
    .syntax unified
    .thumb

    .text
    .align 2
    .global asm_main
    .thumb_func

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
     * Ch11 Port Control and Interrupts (PORT).  Ch 11, p 251 TRM
     */

    .set PORTA_CTRL_BASE, 0x40049000 /* .set sets the value of a variable */
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
     * General purpose input/output (GPIO). Ch 54, p 1747 TRM
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

    .set SIM_SCGC5, 0x40048038
    .set PORTA_ENABLE, (1 << 9)
    .set PORTB_ENABLE, (1 << 10)
    .set PORTC_ENABLE, (1 << 11)
    .set PORTD_ENABLE, (1 << 12)
    .set PORTE_ENABLE, (1 << 13)
    .set SIM_SCGC5_FLAGS, (PORTA_ENABLE | PORTB_ENABLE | PORTC_ENABLE| PORTD_ENABLE |PORTE_ENABLE)

    .set SIM_SCGC4, 0x40048034
    .set SIM_SCGC4_LLWU_BIT, (1 << 28)
    .set LLWU_CS_ADDR, 0x4007C008
    .set LLWU_CS_ACKISO_BIT, (1 << 7)


    .set SIM_CKLDIV1, 0x40048044
    .set OUTDIV1_BY2, (0x2 << 28)
    .set OUTDIV2_BY2, (0x2 << 24)
    .set OUTDIV3_BY2, (0x2 << 20)
    .set OUTDIV4_BY2, (0x2 << 16)
    .set MCGOUTCLK_DIV2, (OUTDIV1_BY2 | OUTDIV2_BY2 | OUTDIV3_BY2 | OUTDIV4_BY2)


    /*
     * OSC Defines
     */

    .set OSC_CR_ADDR, 0x40065000
    .set OSC_ERCLKEN, (1 << 7)

    /*
     * MCG Defines
     */


    .set TRUE, 1
    .set FALSE, 0
    .set BIT_0, (1 << 0)
    .set BIT_1, (1 << 1)
    .set BIT_2, (1 << 2)
    .set BIT_3, (1 << 3)
    .set BIT_4, (1 << 4)
    .set BIT_5, (1 << 5)
    .set BIT_6, (1 << 6)
    .set BIT_7, (1 << 7)
    .set BIT_8, (1 << 8)



    .set MCG_CTRL_BASE, 0x40064000

    .set MCG_C1_OFFSET, 0
    .set MCG_C2_OFFSET, 1
    .set MCG_C3_OFFSET, 2
    .set MCG_C4_OFFSET, 3
    .set MCG_C5_OFFSET, 4
    .set MCG_C6_OFFSET, 5
    .set MCG_STATUS_OFFSET, 6

    .set MCG_C1_IRCLKEN,        (1 << 1)
    .set MCG_C1_CLKS_EXTERNAL,  (0x02 << 6)
    .set MCG_C1_FRDIV_1024,     (0x03 << 3) /* not quite
                                             * the 31 - 39KHz required for FLL
                                             * but not strickly required for
                                             * FBE - pg 542 TRM
                                             */
    .set MCG_C1_CLKS_SHIFT,     6
    .set MCG_C1_CLKS_NUM_BITS,  2
    .set MCG_C1_FBE_MODE,       (MCG_C1_CLKS_EXTERNAL | MCG_C1_FRDIV_1024 | MCG_C1_IRCLKEN)

    .set MCG_C2_RANGE_VHF,      (0x02 << 4)
    .set MCG_C2_HGO_HIGH_GAIN,  (0 << 3)
    .set MCG_C2_EREFS,          (1 << 2)
    .set MCG_C2_FBE_MODE,       (MCG_C2_RANGE_VHF | MCG_C2_HGO_HIGH_GAIN | MCG_C2_EREFS)

    .set MCG_C5_PRDIV_8, 0x18 /* divide by 25: 50MHz/25 = 2  req range 2MHz - 4MHz */

    .set MCG_C6_PLLS_ENABLE, (1 << 6)
    .set MCG_C6_VDIV_50X,    0x1A /* X50  2*50 =  100MHz! */
    .set MCG_C6_VDIV_48X,    0x18 /* X48  2*48 =  96MHz */
    .set MCG_C6_PBE_MODE,    (MCG_C6_PLLS_ENABLE | MCG_C6_VDIV_48X)

    .set MCG_S_OSCINIT_BIT,     (1 << 1)
    .set MCG_S_IREFST_BIT,      (1 << 4)
    .set MCG_S_PLLST_BIT,       (1 << 5)
    .set MCG_S_LOCK_BIT,        (1 << 6)
    .set MCG_S_CLKST_PLL,       (0x03 << 2)
    .set MCG_S_EXTAL_SELECTED,  (0x02 << 2)

    .set MCG_S_FBE_MODE_TEST_MASK, (MCG_S_OSCINIT_BIT  | MCG_S_IREFST_BIT | MCG_S_EXTAL_SELECTED)
    .set MCG_S_FBE_MODE_SET,        (MCG_S_OSCINIT_BIT | MCG_S_EXTAL_SELECTED)
    .set MCG_S_PBE_MODE_SET,        (MCG_S_PLLST_BIT   | MCG_S_LOCK_BIT)

    .set MCG_C1_RESET_VALUE, 0x4
    .set MCG_C2_RESET_VALUE, 0x0
    .set MCG_C5_RESET_VALUE, 0x0
    .set MCG_C6_RESET_VALUE, 0x0
    .set MCG_S_RESET_VALUE,  0x10

    .set WAIT_LOOP_COUNT,       10000 /* some number */

    .set MC_BASE_PTR,       0x4007E000

    /*
     * Business Time...
     */

    /*
     * Enable PORT (Clock Gate)
     * Signal Multiplexing and Signal Descriptions. Ch 10, p 231 TRM
     *     These bits are cleared after any reset, which disables the clock to
     *     the corresponding module to conserve power. Prior to initializing the
     *     corresponding module, set SCGC5[PORTx] in the SIM module to enable
     *     the clock. Before turning off the clock, make sure to disable the
     *     module.
     */

    ldr r0, =SIM_SCGC5_FLAGS
    ldr r1, =SIM_SCGC5
    str r0, [r1]

    /*
     * Configure PORT Controller
     */

    ldr r0, =PORT_CTRL_FLAGS
    ldr r1, =LED_ORANGE_CTRL_ADDR
    ldr r2, =LED_YELLOW_CTRL_ADDR
    ldr r3, =LED_GREEN_CTRL_ADDR
    ldr r4, =LED_BLUE_CTRL_ADDR
    ldr r5, =SW_1_CTRL_ADDR
    ldr r6, =SW_0_CTRL_ADDR
    ldr r7, [r6]
    str r0, [r1]
    str r0, [r2]
    str r0, [r3]
    str r0, [r4]
    str r0, [r5]
    str r0, [r6]

    /*
     * Configure GPIOs
     *
     * Note: 0 = input, 1 = ouput (all pins default hi z inputs).
     */

    ldr r0, =LEDS_MASK                                    /* LEDs = Outputs */
    ldr r1, =PORTA_DATA_DIRECTION_ADDR
    str r0, [r1]

    /*
     * Preset initial delay time
     */

    ldr r9, =0x0ffff


go_PEE:

                  /* We want core at 100MHz, peripherals can't go > 50.
                   * Need at least a 2X divider.  Adjust the MCGOUTCLK dividers
                   * before you go_PEE, otherwise system with take_a_sh!t... */
    ldr r0, =SIM_CKLDIV1
    ldr r2, =MCGOUTCLK_DIV2
    str r2, [r0] /* Your peripherals can't take the friction... */


                  /* Required transitions FEI -> FBE -> PBE -> PEE.  p553 TRM.
                   *  See also example ss24.5.3.1 TRM pg 564. */

                  /* FEI to FBE (FFL Engaged Internal to FFL Engaged External */
    ldr r0, =MCG_CTRL_BASE
    ldr r1, =MCG_C1_FBE_MODE /* Use EXTAL as system clock source, divide by 1024
                              * to get close to FLL required 31-39kHz.
                              * Set Internal Referanc Clock enable to handle
                              * switching back and forth between
                              * internal|external (see pg 561 TRM).  */

    ldr r2, =MCG_C2_FBE_MODE /* Set freq range to very high, high gain modes,
                              * crystal input.  Not sure if this is optimal
                              * but it works. */

                                /* NB: strb for 8 bit MCG access.
                                 * Oherwise system resets and you say, "F#CK!"*/
    strb r2, [r0, MCG_C2_OFFSET]
    strb r1, [r0, MCG_C1_OFFSET]

    ldr r5, =WAIT_LOOP_COUNT
wait_FBE:
                              /* Test status */
    ldrb r6, [r0, MCG_STATUS_OFFSET]
    mov r11, r6
    and r6, MCG_S_FBE_MODE_TEST_MASK
    cmp r6, MCG_S_FBE_MODE_SET /* Look for: crystal initialized,
                                * external source selected and
                                * is MCGOUTCLK reference.
                                */
    beq go_PBE /* FBE Achieved.  Go PBE */

    sub r5, #1
    cbz r5, go_FEI /* Fail safe */

    b wait_FBE

go_PBE:
    ldr r1, =MCG_C5_PRDIV_8
    ldr r2, =MCG_C6_PBE_MODE

    strb r1, [r0, MCG_C5_OFFSET] /* Set PPL Ref Freq */
    strb r2, [r0, MCG_C6_OFFSET] /* Enable PLL at 100 MHz */

    ldr r5, =WAIT_LOOP_COUNT
wait_PBE:
                              /* Test status */
    ldrb r6, [r0, MCG_STATUS_OFFSET]
    and r6, MCG_S_PBE_MODE_SET /* Clock source PLL & Locked ? */
    cmp r6, MCG_S_PBE_MODE_SET
    beq finish_PEE /* PBE Achieved.  Complete PEE transition. */


    sub r5, #1
    cbz r5, go_FEI /* Fail safe */

    b wait_PBE

finish_PEE:
    ldrb r1, [r0, MCG_C1_OFFSET]
    bfc  r1, MCG_C1_CLKS_SHIFT, MCG_C1_CLKS_NUM_BITS
    strb r1, [r0, MCG_C1_OFFSET] /* Use PLL as MCGOUTCLK */


    ldr r5, =WAIT_LOOP_COUNT
wait_PEE:
                              /* Test status */
    ldrb r6, [r0, MCG_STATUS_OFFSET]
    and r6, MCG_S_CLKST_PLL  /* PLL driving MCGOUTCLK ? */
    cmp r6, MCG_S_CLKST_PLL
    beq good_PEE /* PEE Achieved.  Done. */

    sub r5, #1
    cbz r5, go_FEI /* Fail safe */

    b wait_PEE


go_FEI:
                                                /* Failsafe. Go to POR clock. */
    ldr r8, =TRUE
    ldr r0, =MCG_CTRL_BASE
    ldr r1, =MCG_C1_RESET_VALUE
    ldr r2, =MCG_C2_RESET_VALUE
    ldr r3, =MCG_C5_RESET_VALUE
    ldr r4, =MCG_C6_RESET_VALUE

    strb r1, [r0, MCG_C1_OFFSET]
    strb r2, [r0, MCG_C2_OFFSET]
    strb r3, [r0, MCG_C5_OFFSET]
    strb r4, [r0, MCG_C6_OFFSET]


    ldr r5, =WAIT_LOOP_COUNT
wait_FEI:
                              /* Test status */
    ldrb r6, [r0, MCG_STATUS_OFFSET]
    and r6, MCG_S_RESET_VALUE
    cmp r6, MCG_S_RESET_VALUE
    beq main_loop /* FEI Achieved.  Done. */
    sub r5, #1
    cbz r5, die /* You are a complete failure... */
    b wait_FEI


good_PEE:
    ldr r8, =FALSE

main_loop:

/* If you had a good_PEE, the blue LED will flash.
 * If you failed to go_PEE, the system should revert back to POR default,
 * but the blue LED will not flash. */


/* The switch inputs on timing have been disabled */
test_dec_switch:                                 /* Slow down LEDs if pressed */

    ldr r0, =PORTA_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_1_MASK
    ands r1, r2
    bne test_inc_switch
    add r9, 0 /* #0x40000 */

test_inc_switch:                                  /* Speed up LEDs if pressed */

    ldr r0, =PORTE_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_0_MASK
    ands r1, r2
    bne flash_leds
    sub r9, 0 /* #0x20000 */

flash_leds:

    ldr r0, =LEDS_MASK
    ldr r1, =PORTA_SET_REG          /* Turn all LEDs off (LEDs are NEG logic) */
    str r0, [r1]
    bl delay

    ldr r0, =PORTA_CLR_REG
    ldr r1, =LED_ORANGE
    ldr r2, =LED_YELLOW
    ldr r3, =LED_GREEN
    ldr r4, =LED_BLUE
    str r1, [r0]
    bl delay
    str r2, [r0]
    bl delay
    str r3, [r0]
    bl delay

    cmp r8, TRUE  /* If failed to go_PEE, skip the blue LED */
    beq main_loop
    str r4, [r0]
    bl delay

    b main_loop

    /*
     * If main_loop didn't loop forever we'd want to
     * pull the stacked link register into the program counter so we
     * can return to the start code for epilogue processing (if applicable).
     */
die:
    pop { pc }

    .global delay
    .align 2
    .thumb_func

delay:

    push { r0 }
    mov r0, r9

delay_loop:

    subs r0, #1
    bne delay_loop
    pop { r0 }
    bx lr

    .end
