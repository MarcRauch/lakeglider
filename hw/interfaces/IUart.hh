#ifndef GL_HW_INTERFACES_IUART_H_
#define GL_HW_INTERFACES_IUART_H_

#include <stdint.h>

namespace gl::hw {

/**
Interface for UART functionality
*/
class IUart {
 public:
  /**
   * Read all available bytes (less than the given maximum)
   * @param[in] maxBytes Max bytes to read
   * @param[out] dest Buffer to save bytes
   * @returns True if the read was successful
   */
  virtual bool readBytes(uint32_t numBytes, uint8_t* dest) = 0;

  /**
   * Send a buffer of bytes
   * @param[in] data Buffer of data to send
   * @param[in] numBytes Number of bytes to send
   * @returns True if the write was successful
   */
  virtual bool writeBytes(const uint8_t* data, uint32_t numBytes) = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_IUART_H_
