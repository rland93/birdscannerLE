MEMORY
{
  /* nRF52840 Dongle (PCA10059) with Nordic's Open USB DFU bootloader:
   * MBR at 0x0000-0x0FFF, bootloader from 0xE0000. The MBR reserves the
   * first 8 bytes of RAM. */
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xDF000
  RAM   : ORIGIN = 0x20000008, LENGTH = 0x3FFF8
}
