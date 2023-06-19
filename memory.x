MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 28K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 8K

  STORAGE_FLASH (rx) : ORIGIN = 0x08007000, LENGTH = 4K
}

STORAGE_PAGE_A = ORIGIN(STORAGE_FLASH);
STORAGE_PAGE_B = ORIGIN(STORAGE_FLASH) + 2K;
