#include "hw/interfaces/pico/SpiPico.hh"

#include <pico/stdlib.h>

namespace gl::hw {
bool SpiPico::readBytes(std::span<std::byte> data) {
  select();
  if (spi_read_blocking(spiInst, 0x00, reinterpret_cast<uint8_t*>(data.data()), data.size()) != data.size()) {
    deselect();
    return false;
  }
  deselect();
  return true;
}

bool SpiPico::writeBytes(std::span<const std::byte> data) {
  select();
  const bool success =
      spi_write_blocking(spiInst, reinterpret_cast<const uint8_t*>(data.data()), data.size()) == data.size();
  deselect();
  return success;
}

void SpiPico::setInitialized() {
  isInitialized = true;
}

bool SpiPico::writeReadBytes(std::span<const std::byte> dataWrite, std::span<std::byte> dataRead) {
  select();
  const uint32_t numWrite = dataWrite.size();
  if (spi_write_blocking(spiInst, reinterpret_cast<const uint8_t*>(dataWrite.data()), numWrite) != numWrite) {
    deselect();
    return false;
  }
  const uint32_t numRead = dataRead.size();
  if (spi_read_blocking(spiInst, 0x00, reinterpret_cast<uint8_t*>(dataRead.data()), numRead) != numRead) {
    deselect();
    return false;
  }
  deselect();
  return true;
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
