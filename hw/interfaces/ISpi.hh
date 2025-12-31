#ifndef GL_HW_INTERFACES_ISPI_H_
#define GL_HW_INTERFACES_ISPI_H_

#include <cstddef>
#include <span>

namespace gl::hw {

/**
Interface for SPI functionality
*/
class ISpi {
 public:
  /**
   * Request bytes and then read them
   * @param[out] data Buffer to write to. Size determines number of bytes to read.
   * @returns True on success
   */
  virtual bool readBytes(std::span<std::byte> data) = 0;

  /**
   * Send a buffer of bytes
   * @param[in] data Buffer of data to send
   * @returns True if the write was successful
   */
  virtual bool writeBytes(std::span<const std::byte> data) = 0;

  /**
   * Send a buffer of and immediately read into another buffer
   * @param[in] dataWrite Data to write
   * @param[out] dataRead Buffer to read to. number of requested bytes given by size
   * @returns True on success
   */
  virtual bool writeReadBytes(std::span<const std::byte> dataWrite, std::span<std::byte> dataRead) = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_ISPI_H_
