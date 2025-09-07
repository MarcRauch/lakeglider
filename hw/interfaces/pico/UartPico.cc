#include "hw/interfaces/pico/UartPico.hh"

namespace gl::hw {
bool UartPico::readBytes(uint32_t numBytes, uint8_t* dest) {
  if (!uart_is_readable(uartInst)) {
    return false;
  }
  for (uint32_t i = 0; i < numBytes && uart_is_readable(uartInst); i++) {
    dest[i] = static_cast<uint8_t>(uart_getc(uartInst));
  }
  return true;
}

bool UartPico::writeBytes(const uint8_t* data, uint32_t numBytes) {
  uart_write_blocking(uartInst, data, numBytes);
  return true;
}
}  // namespace gl::hw