/*
 * application.s
 *
 * Example UART code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Geoff Chapman
 * May 17 2012
 */

    .syntax unified
    .thumb

    .text
    .align 2
    .global asm_main
    .thumb_func

#include "globalDefs.h"
#include "registers.h"
#include "nvic.h"

asm_main:                                         /* Called by our start code */

    push { lr }

    /*
     * Tower Schematic:
     *
     *   LEDs used for status/debugging:
     *
     *   LED0: PTA11 (active low) (orange)
     *   LED1: PTA28 (active low) (yellow)
     *   LED2: PTA29 (active low) (green)
     *   LED3: PTA10 (active low) (blue)
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

    .set PORTC_CTRL_BASE, 0x4004B000
    .set PIN_16_OFFSET, (4 * 16)
    .set PIN_17_OFFSET, (4 * 17)

    .set LED_ORANGE_CTRL_ADDR, PORTA_CTRL_BASE + PIN_11_OFFSET
    .set LED_YELLOW_CTRL_ADDR, PORTA_CTRL_BASE + PIN_28_OFFSET
    .set LED_GREEN_CTRL_ADDR,  PORTA_CTRL_BASE + PIN_29_OFFSET
    .set LED_BLUE_CTRL_ADDR,   PORTA_CTRL_BASE + PIN_10_OFFSET
    .set SWITCH_1_CTRL_ADDR,   PORTA_CTRL_BASE + PIN_19_OFFSET

    .set UART3_RX_CTRL_ADDR,   PORTC_CTRL_BASE + PIN_16_OFFSET
    .set UART3_TX_CTRL_ADDR,   PORTC_CTRL_BASE + PIN_17_OFFSET

    .set GPIO_ENABLE, (0b001 << 8)
    .set ALT3_ENABLE, (0b011 << 8)
    .set PULL_UP_ENABLE, (1 << 1)
    .set PULL_UP_SELECT, (1 << 0)
    .set IRQ_FALLING,    (0xa << 16)
    .set PORTA_GPIO_CTRL_FLAGS, (GPIO_ENABLE | PULL_UP_ENABLE | PULL_UP_SELECT)
    .set PORTA_IRQ_CTRL_FLAGS,  (GPIO_ENABLE | PULL_UP_ENABLE | PULL_UP_SELECT | IRQ_FALLING)
    .set PORTC_UART_CTRL_FLAGS, (ALT3_ENABLE)


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

    /*
     * SIM Defines
     */

    .set SIM_SCGC4, 0x40048034
    .set UART3_ENABLE, (1 << 13)
    .set SIM_SCGC4_FLAGS, (UART3_ENABLE)

    .set SIM_SCGC5, 0x40048038
    .set PORTA_ENABLE, (1 << 9)
    .set PORTC_ENABLE, (1 << 11)
    .set SIM_SCGC5_FLAGS, (PORTA_ENABLE | PORTC_ENABLE)

    /******************************* Code Start *******************************

    /*
     * Enable (Clock Gate) for PORT A (GPIOs) and PORT C (UART3) 
     */

    ldr r0, =SIM_SCGC5_FLAGS     
    ldr r1, =SIM_SCGC5
    str r0, [r1]

    /*
     * Enable (Clock Gate) for UART3 
     */
    ldr r0, =SIM_SCGC4_FLAGS
    ldr r1, =SIM_SCGC4
    str r0, [r1]

    /*
     * Configure PORT Controller  (GPIOs)
     */

    ldr r0, =PORTA_GPIO_CTRL_FLAGS
    ldr r1, =LED_ORANGE_CTRL_ADDR
    ldr r2, =LED_YELLOW_CTRL_ADDR
    ldr r3, =LED_GREEN_CTRL_ADDR
    ldr r4, =LED_BLUE_CTRL_ADDR
    str r0, [r1]
    str r0, [r2]
    str r0, [r3]
    str r0, [r4]

    ldr r0, =PORTA_IRQ_CTRL_FLAGS
    ldr r1, =SWITCH_1_CTRL_ADDR
    str r0, [r1]

    /*
     * Configure PORT Controller  (UART)
     */
    ldr r0, =PORTC_UART_CTRL_FLAGS
    ldr r1, =UART3_RX_CTRL_ADDR
    ldr r2, =UART3_TX_CTRL_ADDR
    str r0, [r1]
    str r0, [r2]

    /*
     * Configure GPIOs
     *
     * Note: 0 = input, 1 = ouput (all pins default hi z inputs).
     */

    ldr r0, =LEDS_MASK                                      /* LEDs = Outputs */
    ldr r1, =PORTA_DATA_DIRECTION_ADDR
    str r0, [r1]

    /*
     * Configure UART (see notes)
     */

    ldrb r0, =0x00
    ldr  r1, =UART3_BDH
    strb r0, [r1]

    ldrb r0, =0x85
    ldr  r1, =UART3_BDL 
    strb r0, [r1]

    ldrb r0, =0x0b
    ldr  r1, =UART3_C4
    strb r0, [r1]


    /*
     * Configure NVIC 
     * 
     * NVIC Interrupt Set Enable Register    (NVIC_ISERx)
     * NVIC Interrupt Clear Enable Register  (NVIC_ICERx)
     * NVIC Interrupt Set Pending Register   (NVIC_ISPRx)
     * NVIC Interrupt Clear Pending Register (NVIC_ICPRx)
     * NVIC Interrupt Priority Register      (NVIC_IPRx)
     *
     * UART3 uses IRQ 32
     *
     * x   = IRQ / 32
     *     =  51 / 32 = 1
     *                                                  
     * To determine UART3 bitfield location within these registers:
     * bit = IRQ mod 32
     *     =  51 mod 32 = 19
     * 
     */
    
    ldr  r0, =BIT_19
    ldr  r1, =NVIC_ICPR1
    ldr  r2, =NVIC_ISER1
    ldr  r3, =NVIC_PR51  
    str  r0, [r1]                  /* Clear pending interrupts       */
    str  r0, [r2]                  /* Enable interrupts from UART3   */
    ldrb r0, =2
    strb r0, [r3]                  /* Set priority of IRQ 51 to 2    */

    ldr  r0, =BIT_23
    ldr  r1, =NVIC_ICPR2
    ldr  r2, =NVIC_ISER2
    ldr  r3, =NVIC_PR87
    str  r0, [r1]                  /* Clear pending interrupts       */
    str  r0, [r2]                  /* Enable interrupts from switch  */
    ldrb r0, =1
    strb r0, [r3]                  /* Set priority of IRQ 87 to 1    */

    /*
     * Enable UART3 Transmitter, Receiver, Receiver Interrupts
     */

    ldrb r0, =(UART_C2_RE_MASK | UART_C2_TE_MASK | UART_C2_RIE_MASK)
    ldr  r1, =UART3_C2
    strb r0, [r1]

    /*
     * Preset initial delay time
     */

    ldr r9, =0xcffff

    ldr r0, =LEDS_MASK
    ldr r1, =PORTA_SET_REG
    str r0, [r1]                               /* Turn all LEDs off */

main_loop:

    cmp   r7,#0                 /* R7 is debounce timer. If set dec to 0 */
    it    ne
    subne r7,r7,#1

    b main_loop

    pop { pc }


/*******************************************************************************
* Delay function 
*******************************************************************************/
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

/*******************************************************************************
* Switch ISR 
*******************************************************************************/

    .global _switch_isr
    .align 2
    .thumb_func

_switch_isr:
    push { r0, r1, lr }

    add  r5, r5, #1

    ldr  r1, =0x400490A0
    ldr  r0, [r1]
    ands r0, BIT_19
    beq  switch_done 

    cmp r7, #0
    bne switch_done 

    mov r7, #1000
    ldr  r1, =PORTA_TGL_REG        /* Toggle green LED */
    ldr  r0, =LED_GREEN
    str  r0, [r1]

    bl delay

switch_done:
    ldr r1,=0x400490A0             /* Clear Interrupt  */
    ldr r0,=BIT_19
    str r0,[r1]
    pop  { r0, r1, lr }
    bx lr 


/*******************************************************************************
* UART3 Status ISR
*******************************************************************************/

    .global _uart3_status_isr 
    .align 2
    .thumb_func

_uart3_status_isr:
    push { r0, r1, lr }

    ldr  r1, =UART3_S1
    ldrb r0, [r1]
    ands r0, UART_S1_OR_MASK
    beq  getChar 

    ldr  r1, =PORTA_CLR_REG        /* Set orange LED indicated OR condition   */
    ldr  r0, =LED_ORANGE
    str  r0, [r1]

    ldr  r1, =UART3_S1
    ldrb r0, [r1]
    ldr  r1, =UART3_D
    ldrb r0, [r1]                  /* Clear OR flag by reading S1 followd by D*/
    b    done
    
getChar:
    ldr  r1, =UART3_S1
    ldrb r0, [r1]
    ands r0, UART_S1_RDRF_MASK
    beq  done
    
    ldr  r1, =UART3_D
    ldrb r0, [r1]                  /* Clear ISR flag by reading data register */
    strb r0, [r1]                  /* Echo received data                      */


    ldr  r1, =PORTA_SET_REG 
    ldr  r0, =LED_ORANGE
    str  r0, [r1]

    ldr  r1, =PORTA_TGL_REG 
    ldr  r0, =LED_BLUE
    str  r0, [r1]

done:
    pop  { r0, r1, lr }
    bx lr 


    .end
