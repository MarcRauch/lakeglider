#include "hw/interfaces/pico/I2cPico.hh"

namespace gl::hw {

bool I2cPico::readBytes(uint8_t address, uint8_t numBytes, uint8_t* dest) {
  uint8_t bytesRead = i2c_read_timeout_us(i2cInst, address, dest, numBytes, false, 100 + 300 * numBytes);
  return bytesRead == numBytes;
}

bool I2cPico::readRegister(uint8_t address, uint8_t reg, uint8_t numBytes, uint8_t* dest) {
  bool success = writeBytes(address, &reg, 1);
  success &= readBytes(address, numBytes, dest);
  return success;
}

bool I2cPico::writeBytes(uint8_t address, const uint8_t* data, uint8_t numBytes) {
  return i2c_write_timeout_us(i2cInst, address, data, numBytes, false, 100 + 300 * numBytes) == numBytes;
}

bool I2cPico::writeCmd(uint8_t address, uint8_t cmd, const uint8_t* data, uint8_t numBytes) {
  uint8_t writeData[numBytes + 1];
  writeData[0] = cmd;
  for (uint8_t i = 0; i < numBytes; i++) {
    writeData[i + 1] = data[i];
  }
  return writeBytes(address, writeData, numBytes + 1);
}

bool I2cPico::writeCmd(uint8_t address, uint8_t cmd, uint8_t data) {
  return writeCmd(address, cmd, &data, 1);
}

}  // namespace gl::hw