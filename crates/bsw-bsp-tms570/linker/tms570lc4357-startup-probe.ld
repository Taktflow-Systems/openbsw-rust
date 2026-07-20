/*
 * Compile/link/disassembly probe only. Not authorized for target loading.
 * Exact TMS570LC4357 revision-B main flash and SRAM ranges from SPNS195C.
 */
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_tms570_reset)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 4M
  RAM   (rwx) : ORIGIN = 0x08000000, LENGTH = 512K
}

SECTIONS
{
  .vectors ORIGIN(FLASH) : ALIGN(32)
  {
    __vector_start = .;
    KEEP(*(.vectors.tms570))
    __vector_end = .;
  } > FLASH

  .text : ALIGN(4)
  {
    KEEP(*(.startup.tms570))
    KEEP(*(.exceptions.tms570))
    *(.text .text.*)
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH

  .data : ALIGN(4)
  {
    __data_start = .;
    *(.data .data.*)
    . = ALIGN(4);
    __data_end = .;
  } > RAM AT> FLASH
  __data_load = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(4)
  {
    __bss_start = .;
    *(.bss .bss.* COMMON)
    . = ALIGN(4);
    __bss_end = .;
  } > RAM

  .noinit (NOLOAD) : ALIGN(8)
  {
    __noinit_start = .;
    *(.noinit .noinit.*)
    . = ALIGN(8);
    __noinit_end = .;
  } > RAM

  /*
   * The first 160 bytes of SRAM bank 7 are the sole warm-reset retention
   * window. Startup initializes all other bank-7 words with aligned 64-bit
   * stores so their ECC remains valid when hardware auto-init skips the bank.
   */
  .retained 0x08070000 (NOLOAD) : ALIGN(32)
  {
    __retained_start = .;
    KEEP(*(.retained.tms570_exception))
    __retained_end = .;
  } > RAM

  .stacks 0x0807de00 (NOLOAD) : ALIGN(8)
  {
    __stack_region_bottom = .;
    __stack_undefined_bottom = .;
    . += 512;
    __stack_undefined_top = .;
    __stack_abort_bottom = .;
    . += 512;
    __stack_abort_top = .;
    __stack_fiq_bottom = .;
    . += 512;
    __stack_fiq_top = .;
    __stack_irq_bottom = .;
    . += 2048;
    __stack_irq_top = .;
    __stack_supervisor_bottom = .;
    . += 1024;
    __stack_supervisor_top = .;
    __stack_system_bottom = .;
    . += 4096;
    __stack_system_top = .;
    __stack_region_top = .;
  } > RAM

  /DISCARD/ : { *(.ARM.exidx* .ARM.extab* .comment) }
}

ASSERT(__vector_start == 0x00000000, "reset vector is not at address zero")
ASSERT((__vector_end - __vector_start) == 32, "vector table must contain eight A32 words")
ASSERT(__data_end <= __stack_region_bottom, ".data overlaps exception stacks")
ASSERT(__bss_end <= __stack_region_bottom, ".bss overlaps exception stacks")
ASSERT(__noinit_end <= __stack_region_bottom, ".noinit overlaps exception stacks")
ASSERT(__data_end <= __retained_start, ".data overlaps retained record")
ASSERT(__bss_end <= __retained_start, ".bss overlaps retained record")
ASSERT(__noinit_end <= __retained_start, ".noinit overlaps retained record")
ASSERT(__retained_start == 0x08070000, "retained record moved outside bank-7 base")
ASSERT((__retained_end - __retained_start) == 160, "retained record layout changed")
ASSERT((__retained_start % 32) == 0, "retained record lost cache-line alignment")
ASSERT((__retained_end % 8) == 0, "warm ECC initialization is not 64-bit aligned")
ASSERT(__retained_end <= __stack_region_bottom, "retained record overlaps exception stacks")
ASSERT(__stack_region_top == 0x08080000, "exception stacks do not end at SRAM top")
ASSERT((__stack_region_top - __stack_region_bottom) == 8704, "exception stack reservation changed")
ASSERT((LOADADDR(.data) + SIZEOF(.data)) <= (ORIGIN(FLASH) + LENGTH(FLASH)), "flash image exceeds LC4357 main flash")
