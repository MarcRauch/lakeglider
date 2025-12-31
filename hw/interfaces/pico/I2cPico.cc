#include "hw/interfaces/pico/I2cPico.hh"

#include <vector>

namespace gl::hw {

bool I2cPico::readBytes(uint8_t address, std::span<std::byte> data) {
  const uint32_t numBytes = data.size();
  if (i2c_read_timeout_us(i2cInst, address, reinterpret_cast<uint8_t*>(data.data()), numBytes, false,
                          100 + 300 * numBytes) != numBytes) {
    return false;
  }
  return true;
}

bool I2cPico::readRegister(uint8_t address, uint8_t reg, std::span<std::byte> data) {
  if (!writeBytes(address, std::array{static_cast<std::byte>(reg)})) {
    return false;
  }
  return readBytes(address, data);
}

bool I2cPico::writeBytes(uint8_t address, std::span<const std::byte> data) {
  const uint32_t numBytes = data.size();
  return i2c_write_timeout_us(i2cInst, address, reinterpret_cast<const uint8_t*>(data.data()), numBytes, false,
                              100 + 300 * numBytes) == numBytes;
}

bool I2cPico::writeCmd(uint8_t address, uint8_t cmd, std::span<const std::byte> data) {
  const uint32_t numBytes = data.size();
  std::vector<std::byte> writeData;
  writeData.reserve(1 + data.size());
  writeData.push_back(static_cast<std::byte>(cmd));
  writeData.insert(writeData.end(), data.begin(), data.end());
  return writeBytes(address, writeData);
}

bool I2cPico::writeCmd(uint8_t address, uint8_t cmd, uint8_t data) {
  return writeCmd(address, cmd, std::array{static_cast<std::byte>(data)});
}

}  // namespace gl::hw
