/*
 * Linker script for STM32F769NIHx (STM32F769I-Discovery)
 *   2048 KB internal FLASH, 512 KB SRAM (DTCM + SRAM1 + SRAM2), 16 MB external SDRAM
 */

ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size  = 0x2000;
_Min_Stack_Size = 0x2000;

MEMORY
{
  FLASH (rx)       : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM (xrw)        : ORIGIN = 0x20000000, LENGTH = 512K
  /* External SDRAM. The MPU maps the first 8 MB write-through cacheable and the
     rest non-cacheable (see mpu_config() in main.c). */
  SDRAM (rw)       : ORIGIN = 0xC0000000, LENGTH = 16M
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Frame buffers and the LVGL heap (variables placed with
     __attribute__((section(".sdram")))). NOLOAD: not initialised by the start-up
     code and not part of the flash image, since SDRAM only works after
     BSP_SDRAM_Init(). Users must initialise these variables themselves. */
  .sdram (NOLOAD) :
  {
    . = ALIGN(64);
    *(.sdram)
    *(.sdram*)
    . = ALIGN(64);
  } >SDRAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
