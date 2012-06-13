#define __ASM_SRC__ 
#include "arm_cm4.h"
#include "k60n512.h"

    .syntax unified
    .thumb
    .text


    .globl main
    .thumb_func
main:

    bl relocateVectorTable
    bl portAConfig
    bl adcConfig

/***************************************************************************
 *  Main Loop
 *
 *  Use the pot to control the greed led flash rate
 *
 **************************************************************************/

main_loop:

    bl readPot                /* returned in r0 */

    /* apply some gain and update r9 (delay reg) */
    lsl r0, #15
    mov r9, r0

    bl flash_led

    b main_loop

/***************************************************************************
 *  /Main Loop
 **************************************************************************/

    .globl main
    .thumb_func
flash_led:

    push { r0, r1, lr }

#define LED_GREEN BIT_29

    ldr r0, =LED_GREEN

    ldr r1, =PORTA_SET_REG      /* Turn off */
    str r0, [r1]
    bl delay

    ldr r1, =PORTA_CLR_REG      /* Turn on */
    str r0, [r1]
    bl delay

    pop  { r0, r1, lr }
    bx lr



/***************************************************************************
 *  Relocate Vector table to ram
 **************************************************************************/
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
 *  PORTA ISR Handler. Local to this file + installed at runtime
 ****************************************************************************/
    .thumb_func
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
    .thumb_func
isr_porta_pin19:
    push { r0, r1 }
    cmp r7,#0               /* Only toggle LED if debounce counter is 0 */
    it  ne
    bne done

    mov r7,#1000            /* Set switch debounce delay */
    ldr r1,=PORTA_TGL_REG
    ldr r0,=BIT_29
    str r0,[r1]

done:
    ldr r1,=PORTA_ISFR      /* Clear Interrupt */
    ldr r0,=BIT_19
    str r0,[r1]
    pop { r0, r1 }
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

    .thumb_func
    .align 2
portAConfig:

#define PORTA_IRQ_FALLING (0xa << 16)
#define PORTA_GPIO_ENB    BIT_8
#define PORTA_PULL_UP_ENB BIT_1
#define PORTA_PULL_UP_SEL BIT_0
#define PIN19_IRQ_CONFIG  (PORTA_IRQ_FALLING | PORTA_GPIO_ENB       \
                         | PORTA_PULL_UP_ENB | PORTA_PULL_UP_SEL)
#define PIN29_LED_CONFIG  (PORTA_GPIO_ENB    | PORTA_PULL_UP_ENB    \
                                             | PORTA_PULL_UP_SEL)

    push { r0, r1, r2, lr }

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

    ldr r0,=PORTA_DATA_DIRECTION_ADDR
    ldr r1,=BIT_29              /* Set Green LED as output */
    str r1,[r0]


    ldr r0,=isr_porta           /* Add porta isr to ram vector table */
    ldr r1,=_vector_ram
    mov r2,#103                 /* IRQ 87 + 16 Core interrupts */
    str r0, [r1,r2,lsl 2]
                                /* PortA Interrupt is IRQ 87. 32 IRQs per
                                   register which means 23 (87%32) IRQ 
                                   of the  3rd register*/
    pop  { r0, r1, r2, lr }
    bx  lr

    .thumb_func
    .align 2
adcConfig:

    push { r0, r1, r2, r3, r4, lr }
    /********************************************************************
    *
    * ADC Init
    *
    * - Enable Clock to ADC1
    * - Single ended, 8 bit
    * - Configure Low Power, Long Sample Time etc
    * - Calibrate
    * - Write calibration results in PG and MG registers
    *
    ********************************************************************/
#define ADC1_CFG1_BITS (BIT_7 | BIT_4)  /* Low Power, Long Sample Time */


    ldr r0,=SIM_SCGC3            /* Enable Clock to ADC1 */
    ldr r1,=BIT_27
    str r1,[r0]

    /* Configure */
    ldr r0,=ADC1_CFG1             /* load cfg1 address */
    ldr r1, [r0]                  /* read cfg1 */
    ldr r2,=ADC1_CFG1_BITS       
    orr r1,r2 
    str r1,[r0]                   /* write back */

    /* Setup software trigger */
    ldr r0, =ADC1_SC2
    ldr r1, [r0]
    bfc r1, #6,#1               /* software trigger */
    str r1, [r0]

    /* Clear previous failures */
    ldr r0, =ADC1_SC3
    ldr r1, [r0]
    ldr r2, =BIT_6                  /* CALF */
    orr r1, r2                
    str r1, [r0]

    /* Start Calibration */

    ldr r1, =BIT_7                  /* CAL */
    str r1, [r0]
 
    /* COCO is bit 7 which is  already in r1 */
    ldr r0, =ADC1_SC1A

_check_conv_complete:
    ldr r2, [r0]
    and r2, r1
    cmp r2, r1
    bne _check_conv_complete

    /* Check for failures */
    ldr r1, =BIT_6                   /* CALF */
    ands r1, r2
    cbz r1, _cal_passed   

_cal_failed:
    b _cal_failed

_cal_passed:

    /* Need to store the calibration results */
    eor r1, r1                       /* clear r1 */
    ldr r2, =ADC1_CLPS               /* address of first reg */
    ldr r3, =ADC1_CLP0               /* address of last reg */
    
    /* Sum up all the plus side calibrations */
_sum_plus_side:
    ldr r4, [r2],#4
    add r1, r4
    cmp r2, r3
    bne _sum_plus_side

    /* divide by 2 and set 16bit msb */
    lsr r1, r1, #1
    ldr r2, =BIT_15                  /* MSB */
    orr r1, r2  

    /* Store in PG */
    ldr r2, =ADC1_PG
    str r1, [r2]

    /* Sum up all the minus side calibrations */
    eor r1, r1                       /* clear r1 */
    ldr r2, =ADC1_CLMS               /* address of first reg */
    ldr r3, =ADC1_CLM0               /* address of last reg */
_sum_minus_side:
    ldr r4, [r2],#4
    add r1, r4
    cmp r2, r3
    bne _sum_minus_side

    /* divide by 2 and set 16bit msb */
    lsr r1, r1, #1
    ldr r2, =BIT_15                  /* MSB  */
    orr r1, r2  

    /* Store in MG */
    ldr r2, =ADC1_MG
    str r1, [r2]

    pop  { r0, r1, r2, r3, r4, lr }
    bx lr

    .thumb_func
    .align 2
readPot:
    /* value returned in r0 */

    push { r1, r2, lr }
    /* Read pot, AD20 */
    ldr r0, =ADC1_SC1A
    ldr r1, =0x14    
    str r1, [r0]


    /* Check COCO */
    ldr r1, =BIT_7  
_check_conv_complete2:
    ldr r2, [r0]
    and r2, r1
    cmp r2, r1
    bne _check_conv_complete2

    /* read value */
    ldr r1, =ADC1_RA
    ldr r0, [r1]

    pop  { r1, r2, lr }
    bx lr
