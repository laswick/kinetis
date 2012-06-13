/*
 * application.s
 *
 * Example user/application code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Various DMA controller examples / tests
 * 
 * By S. Holford
 * Based on work by Rob Laswick and others
 *
 */

    .syntax unified
    .thumb

    #include "registers.inc"


/*****************************************************************************
 * Data Structures
 *
 */

    .bss
    .align 4
dest_buf1:  .fill  2048,1,0
dest_buf2:  .fill  2048,1,0

    .data

source_buf1: .ascii "abcdefghijklmnop"

source_buf2: .ascii "0123456789ABCDEF"


tcd_test1:  .long source_buf1       /* TCD_SADDR */
            .short 1                /* TCD_SOFF - Increment source address by 1 byte */
            .short 0x0002           /* TCD_ATTR - Source  = 8-bit, dest = 32 bit, no mod */
            .long 16                /* TCD_NBYTES  - 16 Byte minor loop per service */
            .long -16               /* TCD_SLAST - adjustment to source when major is done */
            .long dest_buf1         /* TCD_DADDR */
            .short 4                /* TCD_DOFF - Increment dest address by 4 bytes */
            .short 1                /* TCD_CITER  - Current major iteration count = 1 */
            .long  0                /* TCD_DLASTSGA - adjustment to dest when major is done */
            .short 1                /* TCD_CSR  - Start = 1 */
            .short 1                /* TCD_BITER - Beginning major iteration count = 1 */

tcd_test2:  .long source_buf2       /* TCD_SADDR */
            .short 1                /* TCD_SOFF */
            .short 0x2002           /* TCD_ATTR - Source = 8-bit, mod = 2^4 (16), dest = 32-bit*/
            .long 16                /* TCD_NBYTES */
            .long -256              /* TCD_SLAST - source address adjustment when done major */
            .long dest_buf2         /* TCD_DADDR */
            .short 4                /* TCD_DOFF */
            .short 0x8210           /* TCD_CITER  - Minor loop linking enabled to ourself 16 major loops */
            .long -256              /* TCD_DLASTSGA - dest address adjustment when done major */
            .short 1                /* TCD_CSR */
            .short 0x8210           /* TCD_BITER - Must be same as CITER */

    .text
    .align 4
crlf:       .ascii "\r\n"
            .byte 0
string1:    .ascii "Hello World\r\n"
            .byte 0
sbuf1:      .ascii "Buffer 1:"
            .byte 0
sbuf2:      .ascii "Buffer 2:"
            .byte 0
    .align 4


/*****************************************************************************
 * function delay
 *
 * r9 = number of loops to iterate for delay
 *
 */

    .text
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

/*****************************************************************************
 * function robs_func
 *
 * Original code for reference 
 *
 */

    .text
    .global delay
    .align 2
    .thumb_func

robs_func:

    push { lr }
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

    /*
     * If pressed, force an exception to occur (the following address allows
     * only byte access)...
     */

    ldr r0, =0x40064000
    ldr r1, [r0]

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


/*****************************************************************************
 * function load_tcd
 * 
 * r0 = tcd address
 * r1 = address of 32-byte tcd to load
 */

    .text
    .align 2
    .global load_tcdn
    .thumb_func

load_tcd:
    push {lr}
    ldr r2, [r1, #DMA_TCD_SADDR]
    str r2, [r0, #DMA_TCD_SADDR]
    ldrh r2, [r1, #DMA_TCD_SOFF]
    strh r2, [r0, #DMA_TCD_SOFF]
    ldrh r2, [r1, #DMA_TCD_ATTR]
    strh r2, [r0, #DMA_TCD_ATTR]
    ldr r2, [r1, #DMA_TCD_NBYTES]
    str r2, [r0, #DMA_TCD_NBYTES]
    ldr r2, [r1, #DMA_TCD_SLAST]
    str r2, [r0, #DMA_TCD_SLAST]
    ldr r2, [r1, #DMA_TCD_DADDR]
    str r2, [r0, #DMA_TCD_DADDR]
    ldrh r2, [r1, #DMA_TCD_DOFF]
    strh r2, [r0, #DMA_TCD_DOFF]
    ldrh r2, [r1, #DMA_TCD_CITER]
    strh r2, [r0, #DMA_TCD_CITER]
    ldr r2, [r1, #DMA_TCD_DLASTSGA]
    str r2, [r0, #DMA_TCD_DLASTSGA]
    ldrh r2, [r1, #DMA_TCD_BITER]
    strh r2, [r0, #DMA_TCD_BITER]
    ldrh r2, [r1, #DMA_TCD_CSR]
    bfc  r2, #0, #1                 /* Ensure start is cleared */
    strh r2, [r0, #DMA_TCD_CSR]
    ldrh r2, [r1, #DMA_TCD_CSR]
    strh r2, [r0, #DMA_TCD_CSR]

    pop {pc}

/*****************************************************************************
 * function emit_char
 * 
 * Polled transmit out UART3
 *
 * Sends the byte in r0 out the serial port.
 */

    .text
    .align 2
    .global emit_char
    .thumb_func

emit_char:
    push {lr}
    push {r1,r2}

empty_wait:
    ldr  r1, =UART3_S1
    ldrb r2, [r1]
    ands r2, UART_S1_TDRE
    beq  empty_wait

    ldr  r1, =UART3_D
    strb r0, [r1]

    pop {r1,r2}
    pop {pc}

/*****************************************************************************
 * function emit_string
 * 
 * Transmit the passed string in r0 until a null byte is found
 *
 *
 */

    .text
    .align 2
    .global emit_string
    .thumb_func

emit_string:
    push {lr}
    push {r0,r1}

    mov r1, r0
xloop:
    ldrb r0, [r1], #1
    cmp r0, #0
    beq es_exit

    bl emit_char
    b xloop 

es_exit:
    pop {r0,r1}
    pop {pc}

/*****************************************************************************
 * function dump_buffers
 * 
 * Dump buffer status to serial port
 *
 *
 */

    .text
    .align 2
    .global dump_buffers
    .thumb_func

dump_buffers:
    push {lr}

    ldr r0, =sbuf1
    bl emit_string
    ldr r0, =dest_buf1
    bl emit_string
    ldr r0, =crlf
    bl emit_string

    ldr r0, =sbuf2
    bl emit_string
    ldr r0, =dest_buf2
    bl emit_string
    ldr r0, =crlf
    bl emit_string

    pop {pc}

/*****************************************************************************
 * function asm_main
 * 
 * Entry point called from startup code
 */

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
     * Business Time...
     */

    /*
     * Enable UART3 (Clock Gate)
     */

    ldr r0, =(UART3_ENABLE)
    ldr r1, =SIM_SCGC4
    str r0, [r1]

    /*
     * Enable PORT (Clock Gate)
     */

    ldr r0, =(PORTA_ENABLE | PORTC_ENABLE | PORTE_ENABLE)
    ldr r1, =SIM_SCGC5
    str r0, [r1]

    /*
     * Enable DMA_MUX and PIT (Clock Gate)
     */

    /* ldr r0, =(DMAMUX_ENABLE | PIT_ENABLE) */
    ldr r0, =(DMAMUX_ENABLE)
    ldr r1, =SIM_SCGC6
    str r0, [r1]

    /*
     * Enable eDMA (Clock Gate)
     */

    ldr r0, =(EDMA_ENABLE)
    ldr r1, =SIM_SCGC7
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
     * Configure PORT Controller  (UART)
     */
    ldr r0, =PORTC_UART_CTRL_FLAGS
    ldr r1, =UART3_RX_CTRL_ADDR
    ldr r2, =UART3_TX_CTRL_ADDR
    str r0, [r1]
    str r0, [r2]

    /*
     * Configure UART (see notes)
     */

    ldrb r0, =0x00
    ldr  r1, =UART3_BDH
    strb r0, [r1]

    ldrb r0, =0x0B
    ldr  r1, =UART3_BDL 
    strb r0, [r1]

    ldrb r0, =0x03
    ldr  r1, =UART3_C4
    strb r0, [r1]

    ldrb r0, =0x08
    ldr  r1, =UART3_C2
    strb r0, [r1]

    ldr r0, =string1
    bl emit_string

    bl dump_buffers

dma_1:
    ldr r0, =DMA_TCD0
    ldr r1, =tcd_test1
    bl  load_tcd

    bl dump_buffers


dma_2:
    ldr r0, =DMA_TCD1
    ldr r1, =tcd_test2
    bl  load_tcd

    bl dump_buffers

dma_3:
    nop
    nop

/* Fire up the old PIT  : PIT channel 2 to connect to DMA 2 */

    bl  robs_func 

    pop { pc }


    .end
