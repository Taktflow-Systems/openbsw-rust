/* STM32G474RE — 512 KB Flash, 128 KB SRAM */
MEMORY
{
    /* Top 8 KiB (0x0807E000..0x0807FFFF) belongs exclusively to storage. */
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 504K
    STORAGE (rx) : ORIGIN = 0x0807E000, LENGTH = 8K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 127K
    NOINIT (rwx) : ORIGIN = 0x2001FC00, LENGTH = 1K
}

_noinit_start = ORIGIN(NOINIT);
_storage_start = ORIGIN(STORAGE);
_storage_end = ORIGIN(STORAGE) + LENGTH(STORAGE);
