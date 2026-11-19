MEMORY
{
  /* nRF5340 application core, secure image, no bootloader. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1008K /* top 16K: bond storage, see bonds.rs */
  RAM   : ORIGIN = 0x20000000, LENGTH = 496K /* top 16K: HCI rings shared with the network core, see hci-shm */
}
