/*
 * RAM-only RTI timing role, loaded after the separately proven clock role.
 * Never use this script for flash programming.
 */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_tms570_rti_probe)

MEMORY
{
  RTI_RAM (rwx) : ORIGIN = 0x08076000, LENGTH = 0x100
}

SECTIONS
{
  .rti_probe ORIGIN(RTI_RAM) : ALIGN(4)
  {
    __rti_probe_start = .;
    KEEP(*(.rti_probe))
    __rti_probe_end = .;
  } > RTI_RAM

  .data : { *(.data .data.*) } > RTI_RAM
  .bss (NOLOAD) : { *(.bss .bss.* COMMON) } > RTI_RAM

  /DISCARD/ :
  {
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.ARM.exidx* .ARM.extab*)
    *(.comment .note* .eh_frame*)
  }
}

ASSERT(__rti_probe_start == 0x08076000, "RTI probe entry moved")
ASSERT((__rti_probe_end - __rti_probe_start) <= 0x100, "RTI probe exceeds reserved SRAM window")
ASSERT(SIZEOF(.data) == 0, "RTI probe gained writable data")
ASSERT(SIZEOF(.bss) == 0, "RTI probe gained BSS")
ASSERT(__rti_probe_end <= 0x08076100, "RTI probe exceeded its isolated window")
