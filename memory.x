/* https://github.com/nrf-rs/nrf-hal/blob/master/nrf9160-hal/memory.x */

MEMORY
{
    FLASH        : ORIGIN = 0x00050000, LENGTH = 768K
    RAM   : ORIGIN = 0x20010000, LENGTH = 64K
}