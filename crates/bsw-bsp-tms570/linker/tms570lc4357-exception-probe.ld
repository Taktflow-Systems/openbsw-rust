/* RAM-only retained-exception role. Never use this script for flash programming. */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_tms570_exception_probe)

MEMORY
{
  EXCEPTION_CODE (rx) : ORIGIN = 0x08073000, LENGTH = 0x1000
  EXCEPTION_RAM  (rw) : ORIGIN = 0x08074000, LENGTH = 0x0800
}

SECTIONS
{
  .exception_probe ORIGIN(EXCEPTION_CODE) : ALIGN(4)
  {
    __exception_probe_start = .;
    KEEP(*(.exception_probe))
    KEEP(*(.exception_probe_anchor))
    KEEP(*(.exceptions.tms570))
    KEEP(*(.text.*link_exception_capture*))
    __exception_probe_end = .;
  } > EXCEPTION_CODE

  .retained ORIGIN(EXCEPTION_RAM) (NOLOAD) : ALIGN(32)
  {
    __retained_start = .;
    KEEP(*(.retained.tms570_exception))
    __retained_end = .;
  } > EXCEPTION_RAM

  /DISCARD/ :
  {
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.data .data.*)
    *(.bss .bss.* COMMON)
    *(.ARM.exidx* .ARM.extab*)
    *(.comment .note* .eh_frame*)
  }
}

ASSERT(_tms570_exception_probe == 0x08073000, "exception probe entry moved")
ASSERT(__exception_probe_end <= 0x08074000, "exception code exceeds its SRAM window")
ASSERT(__retained_start == 0x08074000, "physical retained record moved")
ASSERT((__retained_end - __retained_start) == 160, "retained record layout changed")
ASSERT((__retained_start % 32) == 0, "retained record alignment changed")
ASSERT(__retained_end <= 0x08074100, "retained record overlaps undefined stack")
ASSERT(0x08074100 < 0x08074200, "undefined stack range is empty")
ASSERT(0x08074200 < 0x08074400, "abort stack range is empty")
ASSERT(0x08074400 < 0x08074600, "supervisor stack range is empty")
ASSERT(0x08074600 < 0x08074800, "system stack range is empty")
