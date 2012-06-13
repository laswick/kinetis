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

periodic_xmit: .ascii "Pot Code:0x000\r\n"

    .set xm_pos, (periodic_xmit + 11)

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

tcd_test3:  .long activate_code     /* TCD_SADDR - point to the "activate channel 3" code */
            .short 0                /* TCD_SOFF - don't increment source address or dest address */
            .short 0x0000           /* TCD_ATTR - 8-bit source and dest*/
            .long  1                /* TCD_NBYTES */
            .long  0                /* TCD_SLAST - don't mess with address */
            .long  DMA_SERQ         /* TCD_DADDR - point to the SERQ register */
            .short 0                /* TCD_DOFF - leave destination  address alone */
            .short 0x1              /* TCD_CITER - one major loop */
            .long  0                /* TCD_DLASTSGA - no dest address adjust */
            .short 0x0008           /* TCD_CSR - no activation - PIT will do it */
            .short 0x1              /* TCD_BITER - Must be same as CITER */

tcd_test4:  .long periodic_xmit     /* TCD_SADDR - point to the com buffer */
            .short 1                /* TCD_SOFF - byte at a time */
            .short 0x0000           /* TCD_ATTR - 8 bits source and dest, 2^4 mod */
            .long 1                 /* TCD_NBYTES - 1 byte per DMA request (character at a time) */
            .long -16               /* TCD_SLAST - point to start of string when done major */
            .long UART5_D           /* TCD_DADDR - UART5 TX data */
            .short 0                /* TCD_DOFF - no address changes */
            .short 16               /* TCD_CITER  - 16 major loops (chars / DMA requests) */
            .long  0                /* TCD_DLASTSGA - dest address is fixed */
            .short 0x0008           /* TCD_CSR - UART5 TX will trigger - clear activation when done  */
            .short 16               /* TCD_BITER - Must be same as CITER */


    .text
    .align 4
activate_code: .byte 0x03

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
    push {r0,r1,r2}

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

    pop {r0,r1,r2}
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
    /* ldr  r1, =UART5_S1 */
    ldrb r2, [r1]
    ands r2, UART_S1_TDRE
    beq  empty_wait

    ldr  r1, =UART3_D
    /* ldr  r1, =UART5_D */
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
 *
 * sim_config
 *
 * Set up the SIM for this demo. - I.E. Turn on all the needed clock gates
 *
 */

    .text
    .align 2
    .global sim_config
    .thumb_func

sim_config:

    push { r0, r1, lr }
    /*
     * Enable UART5 (Clock Gate)
     */

    /* ldr r0, =(UART5_ENABLE)
    ldr r1, =SIM_SCGC1
    str r0, [r1] */

    /*
     * Enable Clock to ADC1
     */

    ldr r0,=SIM_SCGC3
    ldr r1,=ADC1_ENABLE
    str r1,[r0]

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
    /* ldr r0, =(PORTA_ENABLE | PORTE_ENABLE) */
    ldr r1, =SIM_SCGC5
    str r0, [r1]

    /*
     * Enable DMA_MUX and PIT (Clock Gate)
     */

    /* ldr r0, =(DMAMUX0_ENABLE | DMAMUX1_ENABLE | PIT_ENABLE | FTM0_ENABLE) */
    ldr r0, =(DMAMUX0_ENABLE | PIT_ENABLE | FTM0_ENABLE)
    ldr r1, =SIM_SCGC6
    str r0, [r1]

    /*
     * Enable eDMA (Clock Gate)
     */

    ldr r0, =(EDMA_ENABLE)
    ldr r1, =SIM_SCGC7
    str r0, [r1]


    pop  { r0, r1, lr }
    bx lr

/*****************************************************************************
 *
 * leds_config
 *
 * Set up the GPIO ports to allow LED control
 *
 */

    .text
    .align 2
    .global leds_config
    .thumb_func

leds_config:

    push { r0, r1, lr }

    /*
     * Configure PORT Controller
     */

    ldr r0, =PORT_CTRL_FLAGS
    ldr r1, =LED_ORANGE_CTRL_ADDR
    str r0, [r1]
    ldr r1, =LED_YELLOW_CTRL_ADDR
    str r0, [r1]
    ldr r1, =LED_GREEN_CTRL_ADDR
    str r0, [r1]
    ldr r1, =LED_BLUE_CTRL_ADDR
    str r0, [r1]
    ldr r1, =SW_1_CTRL_ADDR
    str r0, [r1]
    ldr r1, =SW_0_CTRL_ADDR
    str r0, [r1]

    /*
     * Configure GPIOs
     *
     * Note: 0 = input, 1 = ouput (all pins default hi z inputs).
     */

    ldr r0, =LEDS_MASK                                      /* LEDs = Outputs */
    ldr r1, =PORTA_DATA_DIRECTION_ADDR
    str r0, [r1]

    pop  { r0, r1, lr }
    bx lr

/*****************************************************************************
 *
 * adc_config
 *
 * - Enable Clock to ADC1
 * - Single ended, 8 bit
 * - Configure Low Power, Long Sample Time etc
 * - Calibrate
 * - Write calibration results in PG and MG registers
 *
 */

    .text
    .align 2
    .global adc_config
    .thumb_func

adc_config:

    push { r0, r1, r2, r3, r4, lr }


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
    ldr r2, =(1 << 6)                  /* CALF */
    orr r1, r2
    str r1, [r0]

    /* Start Calibration */

    ldr r1, =(1 << 7)                 /* CAL */
    str r1, [r0]

    /* COCO is bit 7 which is  already in r1 */
    ldr r0, =ADC1_SC1A

_check_conv_complete:
    ldr r2, [r0]
    and r2, r1
    cmp r2, r1
    bne _check_conv_complete

    /* Check for failures */
    ldr r1, =(1 << 6)                   /* CALF */
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
    ldr r2, =(1 << 15)                  /* MSB */
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
    ldr r2, =(1 << 15)                  /* MSB  */
    orr r1, r2

    /* Store in MG */
    ldr r2, =ADC1_MG
    str r1, [r2]

    pop  { r0, r1, r2, r3, r4, lr }
    bx lr

/*****************************************************************************
 *
 * read_pot
 *
 * Read the tower board potentiometer
 * value returned in r0
 *
 */

    .text
    .align 2
    .global read_pot
    .thumb_func

read_pot:

    push { r1, r2, lr }
    /* Read pot, AD20 */
    ldr r0, =ADC1_SC1A
    ldr r1, =0x14
    str r1, [r0]


    /* Check COCO */
    ldr r1, =(1 << 7)
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

/*****************************************************************************
 *
 * uart_config
 *
 * Set up the uart for 115,200 baud, 8N1, polled.
 *
 */

    .text
    .align 2
    .global uart_config
    .thumb_func

uart_config:

    push { r0, r1, r2, lr }

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

    /*
     * Configure PORT Controller  (UART)
     */
/*    ldr r0, =PORTE_UART_CTRL_FLAGS
    ldr r1, =UART5_RX_CTRL_ADDR
    ldr r2, =UART5_TX_CTRL_ADDR
    str r0, [r1]
    str r0, [r2] */
    /*
     * Configure UART (see notes)
     */
/*
    ldrb r0, =0x00
    ldr  r1, =UART5_BDH
    strb r0, [r1]

    ldrb r0, =0x0B
    ldr  r1, =UART5_BDL
    strb r0, [r1]

    ldrb r0, =0x03
    ldr  r1, =UART5_C4
    strb r0, [r1]

    ldrb r0, =0x08
    ldr  r1, =UART5_C2
    strb r0, [r1] */

    pop  { r0, r1, r2, lr }
    bx lr

/*****************************************************************************
 *
 * pit_config
 *
 * Set up the pit timer for a 1 second toggle
 *
 */

    .text
    .align 2
    .global pit_config
    .thumb_func

pit_config:

    push { r0, r1, lr }

    ldr r0, =PIT_MCR
    ldr r1, =0
    str r1, [r0]

    /* initial PIT timer interval, = 1 sec */
    ldr r9, =#25000000
    ldr r0, =PIT2_LDVAL
    str r9, [r0]

    ldr r0, =PIT2_TCTRL
    ldr r1, =(TCTRL_TEN)
    str r1, [r0]

    pop  { r0, r1, lr }
    bx lr

/*****************************************************************************
 *
 * ftm_config
 *
 * Set up the ftm timer for the PWM demo.
 *
 * Output and RC servo pulse frame.
 * 20 MSec repeat rate with a pulse that is nominally 1.5us, that can vary
 * from 1us to 2us.
 *
 */

    .text
    .align 2
    .global ftm_config
    .thumb_func

ftm_config:

    push { r0, r1, r2, lr }

/*
 * Configure pin PortA6 as FTM0_CH3
 */

    ldr r0,=FTM0_CH3_CTRL_ADDR
    ldr r1,=PORTA_FTM0_CH3_CTRL_FLAGS
    str r1, [r0]

/*
 * Init the CNT register
 */

    ldr r0,=FTM0_CNT
    ldr r1,=0
    str r1, [r0]

/*
 * Disable the write protection bit
 */

    ldr r0,=FTM0_MODE
    ldr r1,=FTM_MODE_WPDIS
    str r1, [r0]

/*
 * Set up a 51200 modulo with a prescale of 8 gives a PWM period of 20ms
 */

    ldr r0,=FTM0_MOD
    ldr r1,=51200
    str r1, [r0]

/*
 * Zero the initial count
 */

    ldr r0,=FTM0_CNTIN
    ldr r1,=0
    str r1, [r0]

/*
 * Set FTM0 up for clocking from sysclk (20,480,000) with a prescale of 8
 */

    ldr r0,=FTM0_SC
    ldr r1,=(FTM_SC_SYSCLK | FTM_SC_PRE8)
    str r1, [r0]

/*
 * Set up Channel 3 for edge aligned PWM output
 */
    ldr r0,=FTM0_C3SC
    ldr r1,=(FTM_CH_MSB | FTM_CH_ELSB)
    str r1, [r0]

/*
 * Set up Channel 3 compare value for 1.5ms to start
 * This register is what will set the pulse width
 * Using sysclock above, 1ms = 2560 (A00), 2ms = 5120 (1400)
 * so 1.5ms = 3840 (F00) as the nominal start.
 * Our pulse resolution will be 2560 counts, which isn't too bad.
 */

    ldr r0,=FTM0_C3V
    ldr r1,=3840
    str r1, [r0]
/*
 * Load the updated C3V
 */

    ldr r0,=FTM0_PWMLOAD
    ldr r1,=FTM_PWMLOAD_EN
    str r1, [r0]

/*
    ldr r0,=FTM0_SYNCONF
    ldr r1,=FTM_SYNCONF_MODE
    str r1, [r0]
*/

    pop  { r0, r1, r2, lr }
    bx lr

/*****************************************************************************
 *
 * update_string
 *
 * write 12-bit hex number in r0 into the transit buffer as hex
 *
 */

    .text
    .align 2
    .global update_string
    .thumb_func

update_string:
    push { r0, r1, r2, lr }

    ldr r1,=xm_pos

    mov r2,r0
    lsr r2,r2,#8
    and r2,r2,#0xF
    cmp r2, #10
    itee lt
    addlt r2,r2, '0'
    subge r2,r2,  #10
    addge r2,r2, 'A'
    strb r2, [r1], #1

    mov r2,r0
    lsr r2,r2,#4
    and r2,r2,#0xF
    cmp r2, #10
    itee lt
    addlt r2,r2, '0'
    subge r2,r2,  #10
    addge r2,r2, 'A'
    strb r2, [r1], #1

    mov r2,r0
    and r2,r2,#0xF
    cmp r2, #10
    itee lt
    addlt r2,r2, '0'
    subge r2,r2,  #10
    addge r2,r2, 'A'
    strb r2, [r1]

    pop  { r0, r1, r2, lr }
    bx lr

/*****************************************************************************
 *
 * update_pwm
 *
 * Read the pot, scale the value and add it to the base timer
 * value (2560) to give us a pwm pulse range from 1us to 2us based on
 * pot min to max
 *
 */

    .text
    .align 2
    .global update_pwm
    .thumb_func

update_pwm:
    push { r0, r1, lr }

    bl read_pot /* pot value is in r0, unsigned 12-bit is what I've set up */

    bl update_string /* Render the pot value int the send buffer */

    ldr r1,=10  /* 4096 / 1.6 gives us a 2560 count range - Mult by 10, div by 16 */
    mul r0, r0, r1
    lsr r0, r0, #4
    add r0, r0, #2560

/*
 * Write new modulus for CH3 - Will update on the next counter "max"
 * event to give a smooth changeover
 */

    ldr r1,=FTM0_C3V
    str r0, [r1]

    ldr r1,=FTM0_PWMLOAD
    ldr r0,=FTM_PWMLOAD_EN
    str r0, [r1]

    pop  { r0, r1, lr }
    bx lr



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

/*    ldr r0, =0x40064000
    ldr r1, [r0] */

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

/*
 * Read pot, update PWM
 */

    bl update_pwm
/*
 * Initiate a DMA transmit
 */
    ldr r0,=DMA_SERQ
    ldr r1,=#2
    strb r1, [r0]

    b main_loop

    /*
     * If main_loop didn't loop forever we'd want to
     * pull the stacked link register into the program counter so we
     * can return to the start code for epilogue processing (if applicable).
     */
    pop { pc }


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
     * Set up all the peripheral modules starting with the SIM
     */

    bl sim_config
    bl leds_config
    bl uart_config
    bl adc_config
    bl pit_config
    bl ftm_config

    /*
     * Start doing interesting stuff
     */

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

/* Setup the DMA MUX to route PIT (with an always trigger) to DMA2
 * and route the UART5 TX request to DMA3 */

/* Mux off first */


dma_4:
    ldr r0, =DMAMUX0_CHCFG_BASE
    ldr r1, =DMAMUX_SRC_DISABLE
    strb r1, [r0,#2]
    strb r1, [r0,#3]

    ldr r0, =DMA_TCD2
    ldr r1, =tcd_test3
    bl  load_tcd

    ldr r0, =DMA_TCD3
    ldr r1, =tcd_test4
    bl  load_tcd

    ldrb r0, =0x80  /* Set TDMAS to enable DMA for TX */
    ldr  r1, =UART5_C5
    strb r0, [r1]

    ldrb r0, =0x88  /* Set TIE, keep TE to enable DMA for TX */
    ldr  r1, =UART5_C2
    strb r0, [r1]

    ldr r0, =DMAMUX0_CHCFG_BASE
    ldr r1, =(DMAMUX_ENABLE | DMAMUX_SRC_UART5_TX)
    strb r1, [r0,#3]
    ldr r1, =(DMAMUX_ENABLE | DMAMUX_SRC_DMA0_ON)
    strb r1, [r0,#2]


    ldr r0,=DMA_SERQ
    ldr r1, =2 /* enable DMA channel 2 in the ERQ*/
    strb r1, [r0]

    bl  robs_func

    pop { pc }


    .end
