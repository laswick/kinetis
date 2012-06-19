/*
 * application.s
 *
 * Example user/application code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Simple GPIO Demonstration:
 *  -flash the 4 LEDS in sequence
 *  -switch 0 speeds up the flashing
 *  -switch 1 slows down the flashing
 *
 * Rob Laswick
 * April 24 2012
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
    .set SIM_SCGC5_FLAGS, (PORTA_ENABLE | PORTE_ENABLE)

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

    /*
     * Preset initial delay time
     */

    ldr r9, =0xcffff

main_loop:

test_dec_switch:                                 /* Slow down LEDs if pressed */

    ldr r0, =PORTA_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_1_MASK
    ands r1, r2
    bne test_inc_switch
    add r9, #0x40000

test_inc_switch:                                  /* Speed up LEDs if pressed */

    ldr r0, =PORTE_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_0_MASK
    ands r1, r2
    bne flash_leds
    sub r9, #0x20000

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
    str r4, [r0]
    bl delay

    b main_loop

    /*
     * If main_loop didn't loop forever we'd want to
     * pull the stacked link register into the program counter so we
     * can return to the start code for epilogue processing (if applicable).
     */

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
