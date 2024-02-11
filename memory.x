MEMORY {
    /* NOTE 1 K = 1 KiBi = 1024 bytes */
    /* These values correspond to the NRF52832 with SoftDevices S113 7.2.0 */
    FLASH : ORIGIN = 0x00000000 + 112K, LENGTH = 512K - 112K
    RAM   : ORIGIN = 0x20000000 + 18K, LENGTH = 64K - 18K
}
