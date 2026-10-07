/* Everything runs from the on-chip RAM, which is loaded with the program
   when the FPGA is configured (or by $readmemh in simulation). */
OUTPUT_ARCH("riscv")
ENTRY(_start)

__ram_size = 16K;

MEMORY
{
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16K
}

SECTIONS
{
    .text : {
        *(.text.init)
        *(.text .text.*)
    } > RAM
    .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } > RAM
    .data   : { *(.data .data.* .sdata .sdata.*) } > RAM
    .bss (NOLOAD) : ALIGN(4) {
        __bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);
    ASSERT(__stack_top - __bss_end >= 1024, "less than 1 KiB left for the stack")
    /* The RAM image is loaded once, when the FPGA is configured. After a
       reset the program restarts with whatever the RAM holds, so writable
       data must be initialised in code (.bss is cleared by crt0.S). */
    ASSERT(SIZEOF(.data) == 0, "initialised writable data is not allowed")
}
