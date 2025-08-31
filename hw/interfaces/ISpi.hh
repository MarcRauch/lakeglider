#ifndef GL_HW_INTERFACES_ISPI_H_
#define GL_HW_INTERFACES_ISPI_H_

#include <stdint.h>

namespace gl::hw {

/**
Interface for SPI functionality
*/
class ISpi {
 public:
  /**
   * Request bytes and then read them
   * @param[in] numBytes Number of bytes to read
   * @param[out] dest Buffer to save result to
   * @returns True if the read was successful
   */
  virtual bool readBytes(uint8_t numBytes, uint8_t* dest) = 0;

  /**
   * Send a buffer of bytes
   * @param[in] data Buffer of data to send
   * @param[in] numBytes Number of bytes to send
   * @returns True if the write was successful
   */
  virtual bool writeBytes(const uint8_t* data, uint8_t numBytes) = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_ISPI_H_
