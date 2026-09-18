/* Derived from ST's STM32F401CCYX_FLASH.ld (CubeIDE 2021, BSD-3-Clause).
 * STM32F401VC has 256 KiB flash and 64 KiB official SRAM. The final 8 bytes
 * are a dedicated NOLOAD one-shot DFU marker and are unavailable to C data,
 * heap, or stack. */
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K - 8
  DFU_MAGIC (xrw) : ORIGIN = 0x2000FFF8, LENGTH = 8
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
}

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
  .text : {
    . = ALIGN(4); *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .;
  } > FLASH
  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.extab : { . = ALIGN(4); *(.ARM.extab* .gnu.linkonce.armextab.*) . = ALIGN(4); } > FLASH
  .ARM : { . = ALIGN(4); __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; . = ALIGN(4); } > FLASH
  .preinit_array : { . = ALIGN(4); PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); . = ALIGN(4); } > FLASH
  .init_array : { . = ALIGN(4); PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); . = ALIGN(4); } > FLASH
  .fini_array : { . = ALIGN(4); PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); . = ALIGN(4); } > FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) *(.RamFunc) *(.RamFunc*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
  .bss : { . = ALIGN(4); _sbss = .; __bss_start__ = _sbss; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = _ebss; } > RAM
  ._user_heap_stack : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . = . + _Min_Heap_Size; . = . + _Min_Stack_Size; . = ALIGN(8); } > RAM
  .dfu_magic (NOLOAD) : { . = ALIGN(4); KEEP(*(.dfu_magic)) . = ALIGN(4); } > DFU_MAGIC
  /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
