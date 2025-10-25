#include "hw/interfaces/pico/SpiPico.hh"

#include <pico/stdlib.h>

namespace gl::hw {
bool SpiPico::readBytes(uint8_t numBytes, uint8_t* dest) {
  select();
  const bool success = spi_read_blocking(spiInst, 0x00, dest, numBytes) == numBytes;
  deselect();
  return success;
}
bool SpiPico::writeBytes(const uint8_t* data, uint8_t numBytes) {
  select();
  const bool success = spi_write_blocking(spiInst, data, numBytes) == numBytes;
  deselect();
  return success;
}

void SpiPico::setInitialized() {
  isInitialized = true;
}

bool SpiPico::writeReadBytes(const uint8_t* dataWrite, uint8_t* dataRead, uint32_t numWrite, uint32_t numRead) {
  select();
  bool success = spi_write_blocking(spiInst, dataWrite, numWrite) == numWrite;
  success &= spi_read_blocking(spiInst, 0x00, dataRead, numRead) == numRead;
  deselect();
  return success;
}

void SpiPico::select() {
  gpio_put(pinCs, 0);
  __asm volatile(
      "nop\n"
      "nop\n"
      "nop\n");
}

void SpiPico::deselect() {
  __asm volatile(
      "nop\n"
      "nop\n"
      "nop\n");
  gpio_put(pinCs, 1);
}
}  // namespace gl::hw