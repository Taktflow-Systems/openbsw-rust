/* RAM-only VIM IRQ/FIQ role. Never use this script for flash programming. */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_tms570_vim_probe)

MEMORY
{
  VIM_CODE (rx) : ORIGIN = 0x08075000, LENGTH = 0x0C00
}

SECTIONS
{
  .vim_probe ORIGIN(VIM_CODE) : ALIGN(4)
  {
    __vim_probe_start = .;
    KEEP(*(.vim_probe))
    __vim_probe_end = .;
  } > VIM_CODE

  .data : { *(.data .data.*) } > VIM_CODE
  .bss (NOLOAD) : { *(.bss .bss.* COMMON) } > VIM_CODE

  /DISCARD/ :
  {
    *(.text .text.*)
    *(.rodata .rodata.*)
    *(.ARM.exidx* .ARM.extab*)
    *(.comment .note* .eh_frame*)
  }
}

ASSERT(_tms570_vim_probe == 0x08075000, "VIM probe entry moved")
ASSERT(__vim_probe_end <= 0x08075C00, "VIM code overlaps state/stack reservation")
ASSERT(SIZEOF(.data) == 0, "VIM probe gained writable data")
ASSERT(SIZEOF(.bss) == 0, "VIM probe gained BSS")
ASSERT(0x08075C38 <= 0x08075D00, "VIM state overlaps IRQ stack")
ASSERT(0x08075D00 < 0x08075E00, "IRQ stack range is empty")
ASSERT(0x08075E00 < 0x08075F00, "FIQ stack range is empty")
ASSERT(0x08075F00 <= 0x08076000, "VIM role overlaps RTI role")
