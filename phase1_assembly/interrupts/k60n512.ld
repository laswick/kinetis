ENTRY(__isr_reset)
SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
    FLASH  (rx) : ORIGIN = 0x00000000, LENGTH = 512K
    SRAM_L (rwx): ORIGIN = 0x1FFF0000, LENGTH = 64K
    SRAM   (rwx): ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
    .text :
    {
        _text = .;
        _vector_rom = .;
        *(.isr_vector)
        /* Allocate space for entire vector */
        . = _text + 0x400;
        *(.flash_config)

        . = _text + 0x410;
        . = ALIGN(8);
        *(.isr_handlers)
        . = ALIGN(8);
        *(.text*)
        *(.rodata*)
        . = ALIGN(8);
        *(.init*)
        *(.fini*)
        *(.init_array*)
        *(.fini_array*)
        *(.jcr*)
        . = ALIGN(8);
        *(.rom)
        *(.bss.rom)
        _etext = .;
        . = ALIGN(8);
    } > FLASH

    .data :
    {
        _ram_start = .;
        _vector_ram = .;
        KEEP(*(.isr_vector_ram))
        /* Allocate space for entire vector */
        . = _vector_ram + 0x410;

        _wdog_debug  = .;
        . = _wdog_debug + 0x12;
        _ewdog_debug = .;

        . = ALIGN(8);
        _data = .;
        *(.data*)
        *(.sdata*)
        _edata = .;

        . = ALIGN(8);
        _bss = .;
        *(.bss*)
        *(.sbss*)
        _ebss = .;

        . = ALIGN(8);
        _heap_start = .;
    } > SRAM AT >FLASH

    STACK_SIZE = 1K + 32;
    _heap_end = ALIGN(ORIGIN(SRAM) + LENGTH(SRAM) - STACK_SIZE, 8);
    _bootstack_top = ALIGN(ORIGIN(SRAM) + LENGTH(SRAM) - 32, 8);
}
