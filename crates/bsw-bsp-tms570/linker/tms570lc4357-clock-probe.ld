/*
 * RAM-only physical clock qualification role.
 * Never use this script for flash programming.
 */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_tms570_clock_probe)

MEMORY
{
  CLOCK_RAM (rwx) : ORIGIN = 0x08077000, LENGTH = 0x1000
}

SECTIONS
{
  .clock_probe ORIGIN(CLOCK_RAM) : ALIGN(4)
  {
    __clock_probe_start = .;
    KEEP(*(.clock_probe))
    __clock_probe_end = .;
  } > CLOCK_RAM

  .data : { *(.data .data.*) } > CLOCK_RAM
  .bss (NOLOAD) : { *(.bss .bss.* COMMON) } > CLOCK_RAM

  /DISCARD/ :
  {
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.ARM.exidx* .ARM.extab*)
    *(.comment .note* .eh_frame*)
  }
}

ASSERT(__clock_probe_start == 0x08077000, "clock probe entry moved")
ASSERT((__clock_probe_end - __clock_probe_start) <= 0x1000, "clock probe exceeds reserved SRAM window")
ASSERT(SIZEOF(.data) == 0, "clock probe gained writable data")
ASSERT(SIZEOF(.bss) == 0, "clock probe gained BSS")
ASSERT(__clock_probe_end <= 0x08078000, "clock probe overlaps the retained marker window")
