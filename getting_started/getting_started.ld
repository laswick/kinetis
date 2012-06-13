/*
 * getting_started.ld
 *
 * Simplified linker script for the Freescaled Kinetis K60 / ARM Cortex-M4.
 *
 * Rob Laswick
 * April 2012
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
        *(.text*)
    } > FLASH

    _stack_start = ALIGN (ORIGIN(SRAM) + LENGTH(SRAM), 8);
}

