/*
 * basic_startcode.s
 *
 * Simplified startcode for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Rob Laswick
 * April 2012
 */

    .syntax unified
    .thumb

    .extern _stack_start                      /* Defined in our linker script */

    .weak asm_main           /* User defined entry point (in a separate file) */

    .text
    .org 0
    .align 2

vector_table:
    .word _stack_start
    .word _reset_handler
	.word _nmi_handler
	.word _hard_fault_handler
	.word _mem_fault_handler
	.word _bus_fault_handler
	.word _usage_fault_handler

	.org (83 + 16) << 2
	.word _tsi_isr


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


    .align 4
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

    /*
     * Note: The preceeding code must complete at speed before we can start
     *       setting breakpoints and single stepping, hense the provided
     *       label below "first_break" (i.e. (gdb) tb first_break).
     */

first_break:

set_stack_pointer:

    /*
     * The main stack pointer is automatically set to the value stored in
     * address 0x00000000 (which is the first element in the vector_table) by
     * the hardware, so technically the next 3 lines are not required.
     */

    ldr r1, =vector_table
    ldr r2, [r1]
    mov	sp,r2

call_user_asm_code:
    bl asm_main

end_loop:
    b end_loop

	.align 4
_nmi_handler:
	nop
	.align 4
_hard_fault_handler:
	nop
	.align 4
_mem_fault_handler:
	nop
	.align 4
_bus_fault_handler:
	nop
	.align 4
_usage_fault_handler:
	nop
	.align 4
_tsi_isr:
	b _tsi_isr

    .end
