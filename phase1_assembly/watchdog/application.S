#define __ASM_SRC__
#include "arm_cm4.h"
#include "k60n512.h"

    .syntax unified
    .thumb
    .text

#define PORTA_IRQ_FALLING (0xa << 16)
#define PORTA_GPIO_ENB    BIT_8
#define PORTA_PULL_UP_ENB BIT_1
#define PORTA_PULL_UP_SEL BIT_0
#define PIN19_IRQ_CONFIG  (PORTA_IRQ_FALLING | PORTA_GPIO_ENB                  \
                         | PORTA_PULL_UP_ENB | PORTA_PULL_UP_SEL)
#define PIN29_LED_CONFIG  (PORTA_GPIO_ENB    | PORTA_PULL_UP_ENB               \
                                             | PORTA_PULL_UP_SEL)
#define PIN11_LED_CONFIG  (PORTA_GPIO_ENB    | PORTA_PULL_UP_ENB               \
                                             | PORTA_PULL_UP_SEL)
    .align 2
    .global main
    .type   main, %function
    .thumb_func
main:
    ldr r0,=SIM_SCGC5            /* Enable Clock to PORTA */
    ldr r1,=BIT_9
    str r1,[r0]

    ldr r0,=PORTA_CTRL_BASE
    mov r1,#19
    ldr r2,=PIN19_IRQ_CONFIG
    str r2,[r0,r1,lsl 2]        /* Set Pin19 as irq falling edge w pull-up */

    ldr r0,=PORTA_CTRL_BASE
    mov r1,#29
    ldr r2,=PIN29_LED_CONFIG
    str r2,[r0,r1,lsl 2]        /* Set Green LED Pin29 as pull-up */

    ldr r0,=PORTA_CTRL_BASE
    mov r1,#11
    ldr r2,=PIN11_LED_CONFIG
    str r2,[r0,r1,lsl 2]        /* Set Orange LED Pin11 as pull-up */

    ldr r0,=PORTA_DATA_DIRECTION_ADDR
    ldr r1,=(BIT_11 | BIT_29)   /* Set Green + Orange LED as output */
    str r1,[r0]

    bl relocateVectorTable

                                /* Flash Orange LED once on startup */
    ldr r1,=PORTA_CLR_REG
    ldr r0,=BIT_11
    str r0,[r1]
    bl  delayFn
    ldr r1,=PORTA_SET_REG
    ldr r0,=BIT_11
    str r0,[r1]

    ldr r0,=#5000               /* 5 second timeout */
    bl wdogInit

    ldr r0,=isr_porta           /* Add porta isr to ram vector table */
    ldr r1,=_vector_ram
    mov r2,#103                 /* IRQ 87 + 16 Core interrupts */
    str r0, [r1,r2,lsl 2]
                                /* PortA Interrupt is IRQ 87. 32 IRQs per
                                   register which means 23 (87%32) IRQ of the
                                   3rd register*/
    ldr r0,=BIT_23
    ldr r1,=NVIC_ICPR2
    str r0,[r1]                 /* Clear any IRQ Pending */
    ldr r1,=NVIC_ISER2
    str r0,[r1]                 /* Enable IRQ */
    cpsie i                     /* Enable Global Interrupts */

loop:
    bl  delayFn
    ldr r1,=PORTA_TGL_REG
    ldr r0,=BIT_29
    str r0,[r1]
    b    loop


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

/*****************************************************************************
 *  Delay Loop
 ****************************************************************************/
    .thumb_func
delayFn:
    nop
    push { r0, r1, lr }
    ldr r0,=#0x300000
delay_loop:
    sub r0,r0,#1
    cmp r0,#0
    bne delay_loop

    pop  { r0, r1, lr }
    bx lr

/*****************************************************************************
 *  PORTA ISR Handler. Local to this file + installed at runtime
 ****************************************************************************/
    .align 2
    .global isr_porta
    .thumb_func
    .type   isr_porta, %function
isr_porta:
    push { r0, r1, lr }

    ldr r1,=PORTA_ISFR      /* Check which pin on porta triggered this irq */
    ldr r0,[r1]
    and r0, r0, #BIT_19
    it   ge
    blge isr_porta_pin19

    pop  { r0, r1, lr }
    bx  lr

/****************************************************************************
 * PORTA PIN19 specific handler. Toggle LED
 ****************************************************************************/
    .align 2
    .global isr_porta_pin19
    .thumb_func
    .type   isr_porta_pin19, %function
isr_porta_pin19:
    push { r0, r1, lr }

    bl wdogPunch

    ldr r1,=PORTA_ISFR      /* Clear Interrupt */
    ldr r0,=BIT_19
    str r0,[r1]
    pop { r0, r1, lr }
    bx  lr

    .end
