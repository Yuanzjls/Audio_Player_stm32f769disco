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
  RAM (xrw)        : ORIGIN = 0x20000000, LENGTH = 512K   /* DTCM 128K + SRAM1 368K + SRAM2 16K */
  /* External SDRAM. The MPU maps all of it write-through cacheable
     (see mpu_config() in main.c). */
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

  /* DMA buffers (DMA_BUFFER in main.h). First in RAM so they land in DTCM,
     which the CPU never caches: DMA needs no cache maintenance there. NOLOAD:
     not zeroed by the start-up code. */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
  } >RAM
  ASSERT(ADDR(.dma_buffer) + SIZEOF(.dma_buffer) <= 0x20020000, "DMA buffers must fit in DTCM")

  /* The Doom engine's variables (libdoom.a, built with -DWITH_DOOM=ON): about
     300 KB, too much for the internal RAM. SDRAM works only after
     BSP_SDRAM_Init(), so main() copies .doom_data from flash and zeroes
     .doom_bss right after it (doom_memory_init). Empty without Doom. */
  .doom_data :
  {
    . = ALIGN(8);
    __doom_data_start = .;
    *libdoom.a:*(.data .data.*)
    . = ALIGN(8);
    __doom_data_end = .;
  } >SDRAM AT> FLASH
  __doom_data_load = LOADADDR(.doom_data);

  .doom_bss (NOLOAD) :
  {
    . = ALIGN(8);
    __doom_bss_start = .;
    *libdoom.a:*(.bss .bss.* COMMON)
    . = ALIGN(8);
    __doom_bss_end = .;
  } >SDRAM

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
