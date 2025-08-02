#ifndef GL_HW_INTERFACES_MOCK_I2CMOCK_H_
#define GL_HW_INTERFACES_MOCK_I2CMOCK_H_

#include <stdint.h>
#include <unordered_map>
#include <vector>

#include "hw/interfaces/II2c.hh"

namespace gl::hw {

/**
Implements a mock I2c interface for testing. The write commend currently does nothing. Registers need to be set manually
with sample data
*/
class I2cMock : public II2c {
 public:
  bool readBytes(uint8_t address, uint8_t numBytes, uint8_t* dest) override;

  bool readRegister(uint8_t address, uint8_t reg, uint8_t numBytes, uint8_t* dest) override;

  bool writeBytes(uint8_t address, const uint8_t* data, uint8_t numBytes) override;

  bool writeCmd(uint8_t address, uint8_t cmd, const uint8_t* data, uint8_t numBytes) override;

  bool writeCmd(uint8_t address, uint8_t cmd, uint8_t data) override;

  /**
   * Set data at a given address which will later be returned by a read
   * @param[in] address Address for which to set fictional data
   * @param[in] inpData Pointer to first byte of the fictional data
   * @param[in] nBytes Number of bytes to read from inpData
   */
  void setBytes(uint8_t address, uint8_t* inpData, uint32_t nBytes);

 private:
  std::unordered_map<uint8_t, std::vector<uint8_t>> data;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_MOCK_I2CMOCK_H_
