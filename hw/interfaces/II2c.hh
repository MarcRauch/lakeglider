#ifndef GL_HW_INTERFACES_II2C_H_
#define GL_HW_INTERFACES_II2C_H_

#include <stdint.h>

#include <span>

namespace gl::hw {

/**
Interface for I2C functionality
*/
class II2c {
 public:
  /**
   * Request bytes and then read them
   * @param[in] address Address to request bytes from
   * @param[out] data Buffer to write to. Numbers of bytes to read given by size
   * @returns True on successful read
   */
  virtual bool readBytes(uint8_t address, std::span<std::byte>) = 0;

  /**
   * Request bytes by sending a register address and then read them
   * @param[in] address Address to request bytes from
   * @param[in] reg Address of register to read
   * @param[out] data Buffer to read to. number of requested bytes given by size of buffer
   * @returns True on successful read
   */
  virtual bool readRegister(uint8_t address, uint8_t reg, std::span<std::byte> data) = 0;

  /**
   * Send a buffer of bytes
   * @param[in] address Address to send bytes too
   * @param[in] data Buffer of data to send
   * @returns True if the write was successful
   */
  virtual bool writeBytes(uint8_t address, std::span<const std::byte> data) = 0;

  /**
   * Write a command id followed by the command itself with multiple bytes
   * @param[in] address Address to send bytes too
   * @param[in] cmd Command id, usually the register to fill
   * @param[in] data The data to write to that register
   * @returns True if the write was successful
   */
  virtual bool writeCmd(uint8_t address, uint8_t cmd, std::span<const std::byte> data) = 0;

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
