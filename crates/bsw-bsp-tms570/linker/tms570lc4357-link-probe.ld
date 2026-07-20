/*
 * Compile/link probe only. This is not the production or RAM bring-up layout.
 * Exact LC4357 SRAM bounds are from SPNS195C table 6-25.
 */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_start)

MEMORY
{
  SRAM (rwx) : ORIGIN = 0x08000000, LENGTH = 512K
}

SECTIONS
{
  .text :
  {
    KEEP(*(.text._start))
    *(.text .text.*)
    *(.rodata .rodata.*)
    KEEP(*(.target_identity))
  } > SRAM

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > SRAM
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > SRAM

  .data : { *(.data .data.*) } > SRAM
  .bss (NOLOAD) :
  {
    *(.bss .bss.*)
    *(COMMON)
  } > SRAM

  /DISCARD/ :
  {
    *(.comment)
    *(.note*)
  }
}

ASSERT(ADDR(.text) == ORIGIN(SRAM), "link probe entry moved")
ASSERT(_start >= ORIGIN(SRAM), "entry below SRAM")
ASSERT(_start < ORIGIN(SRAM) + LENGTH(SRAM), "entry above SRAM")
ASSERT(. <= ORIGIN(SRAM) + LENGTH(SRAM), "link probe exceeds LC4357 SRAM")
