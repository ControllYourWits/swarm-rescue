/* STM32F407VGTx_FLASH.ld
 * Flash: 1MB @ 0x08000000
 * RAM:   192KB @ 0x20000000
 * CCMRAM: 64KB @ 0x10000000
 */
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x2000;   /* 8KB  */
_Min_Stack_Size = 0x1000;   /* 4KB  */

MEMORY {
    CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
    RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 192K
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
}

SECTIONS {
    .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
    .text : {
        *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
        KEEP(*(.init)) KEEP(*(.fini))
    } >FLASH
    .rodata : { *(.rodata) *(.rodata*) } >FLASH
    .ARM.extab : { *(.ARM.extab*) } >FLASH
    .ARM : { __exidx_start = .; *(.ARM.exidx*); __exidx_end = .; } >FLASH
    .preinit_array : { KEEP(*(.preinit_array*)) } >FLASH
    .init_array    : { KEEP(*(.init_array*)) } >FLASH
    .fini_array    : { KEEP(*(.fini_array*)) } >FLASH
    _sidata = LOADADDR(.data);
    .data : { _sdata=.; *(.data) *(.data*); _edata=.; } >RAM AT>FLASH
    . = ALIGN(4);
    .ccmram : { _sccmram=.; *(.ccmram) *(.ccmram*); _eccmram=.; } >CCMRAM AT>FLASH
    .bss : { _sbss=.; *(.bss) *(.bss*) *(COMMON); _ebss=.; } >RAM
    ._user_heap_stack : {
        . = ALIGN(8);
        PROVIDE(end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM
    /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
