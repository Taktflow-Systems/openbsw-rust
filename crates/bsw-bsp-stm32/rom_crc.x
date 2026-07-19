SECTIONS
{
  .openbsw_rom_crc : ALIGN(4)
  {
    KEEP(*(.openbsw.rom-crc));
  } > FLASH
}
/* cortex-m-rt always defines .text, while a given binary may have no
 * standalone .rodata output section. Keep the expected CRC outside the
 * measured .text bounds but anchor it to the guaranteed section. */
INSERT AFTER .text;

ASSERT(SIZEOF(.openbsw_rom_crc) == 4, "release ROM CRC field must be exactly four bytes");
