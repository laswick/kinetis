/*
 * getting_started.s
 *
 * Simplified startcode for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Rob Laswick
 * April 2012
 */

    .syntax unified
    .thumb

    .extern _stack_start

    .text
    .org 0
    .align 2

vector_table:
    .word _stack_start
    .word _reset_handler

    .org 0x400
    .align 2

kinetis_flash_config_field:
    .word 0x01234567
    .word 0x89abcdef
    .word 0xffffffff
    .byte 0xfe
    .byte 0xff
    .byte 0xff
    .byte 0xff

    .align 2
    .thumb_func

_reset_handler:

unlock_watchdog:
    ldr r6, =0x4005200e
    ldr r0, =0xc520
    strh r0, [r6]
    ldr r0, =0xd928
    strh r0, [r6]

disable_watchdog:
    ldr r6, =0x40052000
    ldr r0, =0x01d2
    strh r0, [r6]

first_break:

set_stack_pointer:
    ldr r1, =vector_table
    ldr r2, [r1]
    mov	sp,r2

end_loop:
    b end_loop

    .end
