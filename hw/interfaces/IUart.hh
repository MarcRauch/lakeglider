#ifndef GL_HW_INTERFACES_IUART_H_
#define GL_HW_INTERFACES_IUART_H_

#include <span>

namespace gl::hw {

/**
Interface for UART functionality
*/
class IUart {
 public:
  /**
   * Read all available bytes (less than the given maximum)
   * @param[out] data Buffer to write to. Size determines max bytes to read
   * @returns True if successful
   */
  virtual bool readBytes(std::span<std::byte> data) = 0;

  /**
   * Send a buffer of bytes
   * @param[in] data Buffer of data to send
   * @returns True if the write was successful
   */
  virtual bool writeBytes(std::span<const std::byte> data) = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_IUART_H_
