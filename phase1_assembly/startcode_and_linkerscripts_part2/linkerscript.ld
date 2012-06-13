/*
 * linkerscript.ld
 *
 * Linker script for the Freescaled Kinetis K60 / ARM Cortex-M4.
 *
 * Rob Laswick
 * May 15 2012
 */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
    .text :
    {
        _text_start = .;
        *(.vector_table)

        . = 0x00000400;
        *(.kinetis_flash_config_field)

        *(.default_handlers)

        *(.text*)
        _text_end = .;
    } > FLASH

    .data :
    {
        _data_start = .;
        *(.data*)
        _data_end = .;
    } > SRAM AT > FLASH

    .bss :
    {
        _bss_start = .;
        *(.bss*)
        _bss_end = .;
        _heap_start = .;
    } > SRAM

    /*
     * The rest of the SRAM is used for the heap and stack.
     * The stack grows down therefore is starts at the end/top of SRAM.
     * The heap grows up and starts at the end of the bss section and
     * ends at the bottom of the stack (defined below).
     */

    STACK_SIZE = 1K;
    _heap_end = ALIGN (ORIGIN(SRAM) + LENGTH(SRAM) - STACK_SIZE, 8);
    _stack_start = ALIGN (ORIGIN(SRAM) + LENGTH(SRAM), 8);
}

