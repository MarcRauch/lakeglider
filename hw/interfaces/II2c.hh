#ifndef GL_HW_INTERFACES_II2C_H_
#define GL_HW_INTERFACES_II2C_H_

#include <stdint.h>

namespace gl::hw {

/**
Interface for I2C functionality
*/
class II2c {
 public:
  /**
   * Request bytes and then read them
   * @param[in] address Address to request bytes from
   * @param[in] numBytes Number of bytes to read
   * @param[out] dest Buffer to save result to
   * @returns True if the read was successful
   */
  virtual bool readBytes(uint8_t address, uint8_t numBytes, uint8_t* dest) = 0;

  /**
   * Request bytes by sending a register address and then read them
   * @param[in] address Address to request bytes from
   * @param[in] reg Address of register to read
   * @param[in] numBytes Number of bytes to read
   * @param[out] dest Buffer to save result to
   * @returns True if the read was successful
   */
  virtual bool readRegister(uint8_t address, uint8_t reg, uint8_t numBytes, uint8_t* dest) = 0;

  /**
   * Send a buffer of bytes
   * @param[in] address Address to send bytes too
   * @param[in] data Buffer of data to send
   * @param[in] numBytes Number of bytes to send
   * @returns True if the write was successful
   */
  virtual bool writeBytes(uint8_t address, const uint8_t* data, uint8_t numBytes) = 0;

  /**
   * Write a command id followed by the command itself with multiple bytes
   * @param[in] address Address to send bytes too
   * @param[in] cmd Command id, usually the register to fill
   * @param[in] data The data to write to that register
   * @param[in] numBytes Number of bytes to write
   * @returns True if the write was successful
   */
  virtual bool writeCmd(uint8_t address, uint8_t cmd, const uint8_t* data, uint8_t numBytes) = 0;

  /**
   * Write a command id followed by the command itself with a single byte
   * @param[in] address Address to send bytes too
   * @param[in] cmd Command id, usually the register to fill
   * @param[in] data The data to write to that register (single byte)
   * @returns True if the write was successful
   */
  virtual bool writeCmd(uint8_t address, uint8_t cmd, uint8_t data) = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_II2c_H_
