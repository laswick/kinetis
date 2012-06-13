/*
 * application.S
 *
 * Example code for the Freescale Kinetis K60 / ARM Cortex-M4
 *
 * PIT timer / DAC example
 *  - Setup PIT timer to flash LEDS and step through the DAC output buffer
 *  - switches increase and decrease timer
 *  - LEDS are toggled in sequence, 1 LED per timer interrupt
 *  - initial time set to 1 sec
 *  - DAC is loaded with 16 values, ramp up and down somewhat like a sine wave
 *
 * Daryl Hillman
 * May 10, 2012
 *
 * based on the GPIO and interrupt demos
 */
#define __ASM_SRC__
#include "arm_cm4.h"
#include "k60n512.h"

    .syntax unified
    .thumb

    .section .rodata
    .align 1
    .globl data_array
data_array:
    .byte  0x55, 0x05, 0x66, 0x06, 0x77, 0x07, 0x88, 0x08, 0x99, 0x09, 0xaa,\
           0x0a, 0xbb, 0x0b, 0xc0, 0x0f, 0xbb, 0x0b, 0xaa, 0x0a, 0x99, 0x09,\
           0x88, 0x08, 0x77, 0x07, 0x66, 0x06, 0x55, 0x05, 0x00, 0x00
data_array_end:


    .text
    .align 2
    .globl main
    .thumb_func
main:

    push { lr }

first_break:
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

    ldr r0, =LEDS_MASK
    ldr r1, =PORTA_SET_REG          /* Turn all LEDs off (LEDs are NEG logic) */
    str r0, [r1]

    /* initial PIT timer interval, = 1 sec */
    ldr r9, =#25000000

Start_PIT:
    /*
     * Enable PIT Clock Gate
     */
    ldr r0, =SIM_SCGC6_FLAGS
    ldr r1, =SIM_SCGC6
    str r0, [r1]

    ldr r0, =PIT0_MCR
    ldr r1, =MCR_FRZ
    str r1, [r0]

    ldr r0, =PIT0_LDVAL
    str r9, [r0]

    ldr r0, =PIT0_TCTRL
    ldr r1, =(TCTRL_TIE | TCTRL_TEN)
    str r1, [r0]

    ldr r5, =#0x0

Start_DAC:
    /*
     * Enable DAC Clock Gate
     */
    ldr r0, =SIM_SCGC2_FLAGS
    ldr r1, =SIM_SCGC2
    str r0, [r1]

    ldr r0, =DAC_CR0
    ldr r1, =DAC_CR0_FLAG
    strb r1, [r0]

    ldr r0, =DAC_CR1
    ldr r1, =DAC_CR1_FLAG
    strb r1, [r0]

    ldr r0, =DAC_CR2
    ldr r1, =DAC_CR2_FLAG
    strb r1, [r0]

    ldr r2, =data_array_end
    ldr r1, =data_array
    ldr r0, =DAC_DAT0L_ADDR

DAC_loop:  /* copy data to buffer */
    ldrb r3, [r1]
    strb r3, [r0]
    add r1, #1
    add r0, #1
    cmp r1, r2
    bne DAC_loop

DAC_loop_end:

    bl relocateVectorTable

    ldr r0,=isr_pit0
    ldr r1,=_vector_ram
    mov r2,#84                  /* IRQ 68 + 16 core interrupts */
    str r0, [r1,r2,lsl 2]
                                /* 68 % 32 = bit 4 */
    ldr r0,=BIT_4
    ldr r1,=NVIC_ICPR2
    str r0,[r1]
    ldr r1,=NVIC_ISER2
    str r0,[r1]
    cpsie i

main_loop:

test_dec_switch:                                 /* Slow down LEDs if pressed */

    ldr r0, =PORTA_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_1_MASK
    ands r1, r2
    bne test_inc_switch
    ldr r3, =#25000000
    cmp r9, r3
    beq test_inc_switch
    add r9, #1000

test_inc_switch:                                  /* Speed up LEDs if pressed */

    ldr r0, =PORTE_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_0_MASK
    ands r1, r2
    bne set_pit_value
    cmp r9, #1000
    beq set_pit_value
    sub r9, #1000

set_pit_value:

    ldr r0, =PIT0_LDVAL
    str r9, [r0]

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
    ldr r0, =#10000

delay_loop:

    subs r0, #1
    bne delay_loop
    pop { r0 }
    bx lr

/*****************************************************************************
 *  Relocate Vector table to ram
 ****************************************************************************/
    .extern _vector_rom
    .extern _vector_ram
    .thumb_func
relocateVectorTable:
    push { r0, r1, r2, r3, lr }

    ldr r0,=_vector_rom       /* Externed from linker script */
    ldr r1,=_vector_ram
    mov r2,#0

copy_loop:
    ldr r3,[r0],#4            /* Copy vectors from rom to ram */
    str r3,[r1],#4
    add r2,r2,#1
    cmp r2,#0x400
    bne copy_loop

    ldr r1,=SCR_VTOR          /* Write offset of vector table from 0x0 */
    ldr r0,=_vector_ram
    str r0,[r1]

    pop  { r0, r1, r2, r3, lr }
    bx lr

    .thumb_func
isr_pit0:
    push { r0, r1, lr }

DAC:   /* trigger DAC, buffer read ptr will shift to new value */
    ldr r0,=DAC_CR0
    ldr r1,=DAC_CR0_TRGF
    strb r1, [r0]

    /* toggle LEDS in sequence */
    ldr r0, =PORTA_TGL_REG
    ldr r1, =LED_ORANGE
    cmp r5,#0x00
    beq toggle_led
    ldr r1, =LED_YELLOW
    cmp r5,#0x01
    beq toggle_led
    ldr r1, =LED_GREEN
    cmp r5,#0x02
    beq toggle_led
    ldr r1, =LED_BLUE
toggle_led:
    str r1, [r0]
    add r5,#1
    and r5,#0x03    /* limit index 0 - 3 */

    /* clear PIT event flag */
    ldr r0, =PIT0_TFLG
    ldr r1, =TFLG_TIF
    str r1, [r0]

    /* clear interrupt flag */
    ldr r0,=BIT_4
    ldr r1,=NVIC_ICPR2
    str r0,[r1]

    pop  { r0, r1, lr }
    bx  lr

    .end

