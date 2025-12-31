#include "hw/interfaces/pico/UartPico.hh"

namespace gl::hw {
bool UartPico::readBytes(std::span<std::byte> data) {
  if (!uart_is_readable(uartInst)) {
    return false;
  }
  for (uint32_t i = 0; i < data.size() && uart_is_readable(uartInst); i++) {
    data[i] = std::bit_cast<std::byte>(uart_getc(uartInst));
  }
  return true;
}

bool UartPico::writeBytes(std::span<const std::byte> data) {
  uart_write_blocking(uartInst, reinterpret_cast<const uint8_t*>(data.data()), data.size());
  return true;
}
}  // namespace gl::hw
