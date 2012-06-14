/*
 * application.s
 *
 * Example user/application code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Simple TSI Demonstration:
 *  -flash the 4 LEDS in sequence
 *  -SW1 calibrates the touch pads
 *  -touching a pad quickly causes the LED to toggle once
 *  -touching and holding on a pad causes the LED to toggle continuously
 *
 * Neil Henderson
 * June 7 2012
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
     */

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
    .set PORTE_INPUT_REG,           PORTE_BASE_ADDR + 0x10

    .set SW_0_MASK, (1 << 26)

    /*
     * SIM Defines
     */

    .set SIM_SCGC5, 0x40048038
    .set PORTA_ENABLE, (1 << 9)
    .set PORTE_ENABLE, (1 << 13)
	.set TSI_ENABLE, (1 << 5)
    .set SIM_SCGC5_FLAGS, (PORTA_ENABLE | PORTE_ENABLE | TSI_ENABLE)

	.set TSI0_GENCS , 0x40045000
	.set TSI0_SCANC , 0x40045004
	.set TSI0_PEN   , 0x40045008
	.set TSI0_STATUS, 0x4004500C
	.set TSI0_CNTR_BASE , 0x40045100
	.set TSI0_THLD_BASE , 0x40045120

	.set TSI0_COUNT_LH, 0x16001A00

	.set TSI0_BLUE_PEN, 9  /* schematic p 7 */
	.set TSI0_GREEN_PEN, 7
	.set TSI0_YELLOW_PEN, 8
	.set TSI0_ORANGE_PEN, 5

	.set TSI0_BLUE, (1 << TSI0_BLUE_PEN)
	.set TSI0_GREEN,(1 << TSI0_GREEN_PEN)
	.set TSI0_YELLOW,(1 << TSI0_YELLOW_PEN)
	.set TSI0_ORANGE,(1 << TSI0_ORANGE_PEN)
	.set SCNIP , (1 << 9) /* Scan in progress, test in GENCS before updating regs */
	.set TSIEN, (1 << 7) /* TSI enable, must be disabled prior to reg updates */
	.set NSCN, (7 << 19) /* 8 times per electrode */
	.set PS, (2 << 16)
	.set SWTS, (1 << 8)
	.set SCAN_DIFF, 0x200
	.set TSI0_GENCS_FLAGS, (TSIEN | NSCN | PS) /* enable, prescaler p 1764 */
	.set TSI0_SCANC_FLAGS, 0x48030000 /* p 1768 */
/*	.set TSI0_PEN_FLAGS, (TSI0_ORANGE | TSI0_YELLOW | TSI0_GREEN | TSI0_BLUE)*/
	.set TSI0_PEN_FLAGS, (TSI0_YELLOW | TSI0_GREEN | TSI0_BLUE)


    /*
     * Business Time...
     */

    /*
     * Enable PORT (Clock Gate)
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

    ldr r0, =LEDS_MASK                                      /* LEDs = Outputs */
    ldr r1, =PORTA_DATA_DIRECTION_ADDR
    str r0, [r1]

	/* Configure TSI */
	ldr r0, =TSI0_SCANC
	ldr r1, =TSI0_SCANC_FLAGS
	str r1, [r0]
	ldr r0, =TSI0_PEN
	ldr r1, =TSI0_PEN_FLAGS
	str r1, [r0]
	ldr r1, =TSI0_COUNT_LH
	ldr r0, =TSI0_THLD_BASE
	mov r2, #TSI0_BLUE_PEN
	mov r3, #TSI0_GREEN_PEN
	mov r4, #TSI0_YELLOW_PEN
/*
	Note: the orange touch pad on my Kinetis board seems to be stuck.
	This causes the board to go in to reset so I don't enable orange.
*/
/*	mov r5, #TSI0_ORANGE_PEN*/
	str r1,[r0, r2, lsl 2]
	str r1,[r0, r3, lsl 2]
	str r1,[r0, r4, lsl 2]
/*	str r1,[r0, r5, lsl 2]*/
touchtest:
	ldr r0,=TSI0_STATUS
	ldr r1,=0xFFFFFFFF
	str r1,[r0]
	ldr r0, =TSI0_GENCS
	ldr r1, =TSI0_GENCS_FLAGS
	str r1, [r0]
    /*
     * Preset initial delay time
     */

    ldr r9, =0xcffff * 2


main_loop:
	bl startScan
	bl waitScan
	bl delay
	bl updateLeds
	bl testSw1
	b main_loop

    .global startScan
    .align 2
    .thumb_func

startScan:
    push { r0,r1 }
	ldr r0,=TSI0_STATUS
	ldr r1,=0xFFFFFFFF
	str r1,[r0]
	ldr r0, =TSI0_GENCS
	ldr r1,[r0]
	orr r1,#SWTS
	str r1,[r0]
    pop { r0,r1 }
    bx lr

waitScan:
    push { r0,r1 }
	ldr r0,=TSI0_STATUS
waitScan_loop:
	ldr r1,[r0]
	ands r1,#SCNIP
	bne waitScan_loop
    pop { r0,r1 }
    bx lr

updateLeds:
	push {r0,r1,r2,lr}
	ldr r0,=TSI0_STATUS
	ldr r1,[r0]
	ldr r2, =LED_BLUE
	tst r1,#TSI0_BLUE
	it ne
	blne toggleLed
	ldr r2, =LED_GREEN
	tst r1,#TSI0_GREEN
	it ne
	blne toggleLed
	ldr r2, =LED_YELLOW
	tst r1,#TSI0_YELLOW
	it ne
	blne toggleLed
	ldr r2, =LED_ORANGE
	tst r1,#TSI0_ORANGE
	it ne
	blne toggleLed
	pop  {r0,r1,r2,lr}
	bx  lr

toggleLed:
	push { r0,lr}
	ldr r0, =PORTA_TGL_REG
	str r2,[r0]
	pop  { r0,lr}
	bx lr

testSw1:
	push {r0,r1,r2,lr}

    ldr r0, =PORTA_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_1_MASK
    ands r1, r2
    bne testSw1_exit
	bl startScan
	bl waitScan
	bl delay
	bl updateNoTouch
testSw1_exit:
	pop  {r0,r1,r2,lr}
	bx  lr

updateNoTouch:
	push {r0,r1,r2,r3,lr}
	ldr r1,=TSI0_CNTR_BASE
	mov r2, #TSI0_BLUE_PEN
	ldrh r0,[r1,r2, lsl 1]
	sub r0,#SCAN_DIFF
	lsl r3,r0,#16
	add r0,#SCAN_DIFF * 2
	orr r0,r3
	ldr r1, =TSI0_THLD_BASE
	str r0,[r1,r2, lsl 2]
	ldr r1,=TSI0_CNTR_BASE
	mov r2, #TSI0_GREEN_PEN
	ldrh r0,[r1,r2, lsl 1]
	sub r0,#SCAN_DIFF
	lsl r3,r0,#16
	add r0,#SCAN_DIFF * 2
	orr r0,r3
	ldr r1, =TSI0_THLD_BASE
	str r0,[r1,r2, lsl 2]
	ldr r1,=TSI0_CNTR_BASE
	mov r2, #TSI0_YELLOW_PEN
	ldrh r0,[r1,r2, lsl 1]
	sub r0,#SCAN_DIFF
	lsl r3,r0,#16
	add r0,#SCAN_DIFF * 2
	orr r0,r3
	ldr r1, =TSI0_THLD_BASE
	str r0,[r1,r2, lsl 2]
	ldr r1,=TSI0_CNTR_BASE
	mov r2, #TSI0_ORANGE_PEN
	ldrh r0,[r1,r2, lsl 1]
	sub r0,#SCAN_DIFF
	lsl r3,r0,#16
	add r0,#SCAN_DIFF * 2
	orr r0,r3
	ldr r1, =TSI0_THLD_BASE
	str r0,[r1,r2, lsl 2]
	pop  {r0,r1,r2,r3,lr}
	bx  lr


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
