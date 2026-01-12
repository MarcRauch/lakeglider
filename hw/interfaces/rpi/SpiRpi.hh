#ifndef GL_HW_INTERFACES_RPI_SPIRPI_H_
#define GL_HW_INTERFACES_RPI_SPIRPI_H_

#include <string>

#include "hw/interfaces/ISpi.hh"

namespace gl::hw {

/**
Implements the SPI interface for the Raspberry Pi
*/
class SpiRpi : public ISpi {
 public:
  /**
   * Constructs a SPI instance for the Raspberry Pi
   * @param[in] path Path to SPI device
   * @returns SPI object
   */
  SpiRpi(const std::string& path);
  ~SpiRpi();

  bool writeReadBytes(std::span<const std::byte> dataWrite, std::span<std::byte> dataRead) override;
  bool readBytes(std::span<std::byte> data) override;
  bool writeBytes(std::span<const std::byte> data) override;

 private:
  int fd;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_RPI_SPIRPI_H_
