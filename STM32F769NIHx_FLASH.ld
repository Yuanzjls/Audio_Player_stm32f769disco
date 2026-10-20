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
  /* SDRAM 0xC0000000..0xC0FFFFFF: LCD frame buffers live at 0xC0400000/0xC0800000,
     the STemWin heap (aMemory in GUIConf.c) is placed at 0xC0C00000 */
  GUI_Memory (rw)  : ORIGIN = 0xC0C00000, LENGTH = 4M
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

  /* STemWin heap in external SDRAM. NOLOAD: not initialised by startup code and
     not part of the flash image (SDRAM is only valid after BSP_SDRAM_Init). */
  .GUI_NUMBYTES_section (NOLOAD) :
  {
    . = ALIGN(4);
    *(.GUI_NUMBYTES_section)
    . = ALIGN(4);
  } >GUI_Memory

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
