/*
 * linkerscript_boot.ld
 *
 * Linker script for the Freescaled Kinetis K60 / ARM Cortex-M4.
 * Executes bootloader code out of RAM.
 *
 * Rob Laswick
 * May 15 2012
 */

MEMORY
{
    CODE   (rx) : ORIGIN = 0x00000000, LENGTH = 28K
    FLASH  (rx) : ORIGIN = 0x00007000, LENGTH =  4K
    SRAM_L(rwx) : ORIGIN = 0x1FFF0000, LENGTH = 64K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
    .entry :
    {
        _vector_rom = .;
        KEEP(*(.vector_table))

        . = _vector_rom + 0x400;
        KEEP(*(.kinetis_flash_config_field))

        KEEP(*(.default_handlers))
    } > CODE

    .text :
    {
        . = ALIGN(4);
        _text_load = LOADADDR(.text);
        _text_start = .;
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _text_end = .;
    } >SRAM_L AT>CODE

    .ramcode :
    {
        . = ALIGN(4);
        _ramcode_load = LOADADDR(.ramcode);
        _ramcode_start = .;
        *(.ramcode*)
        . = ALIGN(4);
        _ramcode_end = .;
    } > SRAM_L AT>CODE

    .ramvect :
    {
        . = ALIGN(4);
        _vector_ram_start = .;
        . = _vector_ram_start + 0x400;
        _vector_ram_end = .;
    } > SRAM

    .data :
    {
        . = ALIGN(4);
        _data_load  = LOADADDR(.data);
        _data_start = .;
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    }  > SRAM AT>CODE

    .bss :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss*)
        . = ALIGN(4);
        _bss_end = .;
        _heap_start = .;
    } > SRAM

    /*
     * Allocate the last flash sector for application data storage.
     * Must be the last flash section defined otherwise code may be
     * placed within the flash sector
     */
    .nvStorage :
    {
        _nvStorage_start = .;
        *(.nvStorage*)
    } > FLASH

    /*
     * The rest of the SRAM is used for the heap and stack.
     * The stack grows down therefore is starts at the end/top of SRAM.
     * The heap grows up and starts at the end of the bss section and
     * ends at the bottom of the stack (defined below).
     */

    STACK_SIZE = 1K;
    _heap_end = ALIGN (ORIGIN(SRAM) + LENGTH(SRAM) - STACK_SIZE, 8);
    _stack_start = ALIGN (ORIGIN(SRAM) + LENGTH(SRAM), 8);
    _stack_end = _stack_start - STACK_SIZE;
    _app_code_start = ORIGIN(FLASH) + LENGTH(FLASH);
    _flash_swap_addr = 0xffffffff;  /* Not Supported */
}

