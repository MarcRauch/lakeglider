#include "hw/interfaces/mock/I2cMock.hh"

namespace gl::hw {
bool I2cMock::readBytes(uint8_t address, uint8_t numBytes, uint8_t* dest) {
  if (!(data.find(address) != data.end())) {
    return false;
  }
  memcpy(dest, data[address].data(), numBytes);
  return true;
}

bool I2cMock::readRegister(uint8_t address, uint8_t reg, uint8_t numBytes, uint8_t* dest) {
  // TODO: Use also reg
  return readBytes(address, numBytes, dest);
}

bool I2cMock::writeBytes(uint8_t address, const uint8_t* data, uint8_t numBytes) {
  return true;
}

bool I2cMock::writeCmd(uint8_t address, uint8_t cmd, const uint8_t* data, uint8_t numBytes) {
  return true;
}

bool I2cMock::writeCmd(uint8_t address, uint8_t cmd, uint8_t data) {
  return true;
}

void I2cMock::setBytes(uint8_t address, uint8_t* inpData, uint32_t nBytes) {
  data[address] = std::vector<uint8_t>(nBytes);
  memcpy(data[address].data(), inpData, nBytes);
}
}  // namespace gl::hw
