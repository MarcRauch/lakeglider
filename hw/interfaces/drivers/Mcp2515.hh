#ifndef GL_HW_INTERFACES_DRIVERS_MCP2515_H_
#define GL_HW_INTERFACES_DRIVERS_MCP2515_H_

#include <array>
#include <vector>

#include "hw/Pins.hh"
#include "hw/interfaces/ISpi.hh"

namespace gl::hw {
/**
Interface for Can communication using the MCP2515 according to
https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf and
https://github.com/hoylabs/MCP2515
*/
class Mcp2515 {
 public:
  /**
   * Create a Mcp2515 can controller object
   * @param[in] spi Spi interface connected to the Mcp2515
   * @param[in] csPin Chip select connection to the Mcp2515
   * @param[in] subscriptions Vector of canids to receive
   * @returns Mcp2515 object
   */
  Mcp2515(ISpi* spi, PinGpioSensor csPin, CanId canId, std::vector<CanId> subscriptions);

  /**
   * Send a dataframe
   * @param[in] data Dataframe to send
   * @param[in] len Number of bytes to send
   */
  void send(const std::array<uint8_t, 8>* data, uint8_t len);

  /**
   * @param[in] data Array to save data to
   * @returns Number of bytes read
   */
  uint8_t read(std::array<uint8_t, 8>* data);

 private:
  ISpi* spi;
  const PinGpioSensor csPin;
  const CanId canId;
  const std::vector<CanId> subscriptions;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_DRIVERS_MCP2515_H_
