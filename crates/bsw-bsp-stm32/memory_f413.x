/* STM32F413ZH — 1.5 MB Flash, 320 KB SRAM */
MEMORY
{
    /* Sectors 14-15 (0x08140000..0x0817FFFF) belong only to storage. */
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1280K
    STORAGE (rx) : ORIGIN = 0x08140000, LENGTH = 256K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 319K
    NOINIT (rwx) : ORIGIN = 0x2004FC00, LENGTH = 1K
}

_noinit_start = ORIGIN(NOINIT);
_storage_start = ORIGIN(STORAGE);
_storage_end = ORIGIN(STORAGE) + LENGTH(STORAGE);

ASSERT(ORIGIN(STORAGE) == 0x08140000, "F413 storage origin changed");
ASSERT(LENGTH(STORAGE) == 0x00040000, "F413 storage size changed");
ASSERT(ORIGIN(NOINIT) + LENGTH(NOINIT) == ORIGIN(RAM) + LENGTH(RAM) + LENGTH(NOINIT), "F413 NOINIT must remain at top of SRAM");
