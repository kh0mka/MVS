/* Entry Point */
ENTRY(ResetISR)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        *(.eh_frame)
	_etext = .;
    } > FLASH
    
    .ARM :  AT (ADDR(.text) + SIZEOF(.text))
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    /* used by the startup to initialize data */
    _sidata = .;

    .data : AT ( _sidata )
    {
	. = ALIGN(4);
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
	. = ALIGN(4);
        _sbss = .;
        *(COMMON)
        *(.bss*)
        _ebss = .;
    } > SRAM

    .ccmem :
    {
        KEEP(*(.ccm))
    } > CCMRAM
}