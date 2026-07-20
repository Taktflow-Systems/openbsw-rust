/*
 * RAM-only, register-only physical bring-up role.
 * Never use this script for flash programming.
 */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_tms570_ram_marker)

MEMORY
{
  MARKER_RAM (rwx) : ORIGIN = 0x08078000, LENGTH = 0x100
}

SECTIONS
{
  .ram_marker ORIGIN(MARKER_RAM) : ALIGN(4)
  {
    __ram_marker_start = .;
    KEEP(*(.ram_marker))
    __ram_marker_end = .;
  } > MARKER_RAM

  .data : { *(.data .data.*) } > MARKER_RAM
  .bss (NOLOAD) : { *(.bss .bss.* COMMON) } > MARKER_RAM

  /DISCARD/ :
  {
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.ARM.exidx* .ARM.extab*)
    *(.comment .note* .eh_frame*)
  }
}

ASSERT(__ram_marker_start == 0x08078000, "RAM marker entry moved")
ASSERT((__ram_marker_end - __ram_marker_start) == 12, "RAM marker must remain three A32 instructions")
ASSERT(SIZEOF(.data) == 0, "RAM marker gained writable data")
ASSERT(SIZEOF(.bss) == 0, "RAM marker gained BSS")
ASSERT(__ram_marker_end <= (ORIGIN(MARKER_RAM) + LENGTH(MARKER_RAM)), "RAM marker exceeds reserved window")
