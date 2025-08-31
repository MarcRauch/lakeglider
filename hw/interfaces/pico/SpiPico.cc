#include "hw/interfaces/pico/SpiPico.hh"

#include <pico/stdlib.h>

namespace gl::hw {
bool SpiPico::readBytes(uint8_t numBytes, uint8_t* dest) {
  select();
  spi_write_blocking(spiInst, dest, numBytes);
  deselect();
}
bool SpiPico::writeBytes(const uint8_t* data, uint8_t numBytes) {
  select();
  spi_write_blocking(spiInst, data, numBytes);
  deselect();
}

void SpiPico::setInitialized() {
  isInitialized = true;
}

void SpiPico::select() {
  gpio_put(pinCs, 0);
  __asm volatile(
      "nop\n"
      "nop\n"
      "nop\n");
}

void SpiPico::deselect() {
  gpio_put(pinCs, 1);
  __asm volatile(
      "nop\n"
      "nop\n"
      "nop\n");
}
}  // namespace gl::hw