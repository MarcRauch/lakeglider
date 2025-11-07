#ifndef GL_HW_INTERFACES_DRIVERS_MCP2515_H_
#define GL_HW_INTERFACES_DRIVERS_MCP2515_H_

#include <array>
#include <memory>
#include <vector>

#include "hw/Pins.hh"
#include "hw/interfaces/ISpi.hh"
#include "utils/time/IClock.hh"

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
   * @param[in] clock Clock interface to introduce delays
   * @param[in] subscriptions Vector of canids to receive
   * @returns Mcp2515 object
   */
  Mcp2515(std::shared_ptr<ISpi> spi, std::shared_ptr<utils::IClock> clock, CanId canId,
          const std::vector<CanId>& subscriptions);

  /**
   * Send a dataframe
   * @param[in] data Dataframe to send
   * @param[in] len Number of bytes to send
   * @returns True if send was succesful
   */
  bool send(const std::array<uint8_t, 8>& data, uint8_t len);

  /**
   * @param[in] data Array to save data to
   * @returns Number of bytes read and the id of the sender
   */
  std::pair<uint8_t, CanId> read(std::array<uint8_t, 8>* data);

 private:
  uint8_t readRegister(uint8_t addr);
  void readRegisters(uint8_t addr, uint8_t* data, uint8_t len);
  void writeRegister(uint8_t addr, uint8_t val);
  void writeRegisters(uint8_t addr, const uint8_t* data, uint8_t len);
  void modifyRegister(uint8_t addr, uint8_t mask, uint8_t val);
  void setMode(const uint8_t mode);

  std::shared_ptr<ISpi> spi;
  std::shared_ptr<utils::IClock> clock;

  const CanId canId;
  const std::vector<CanId> subscriptions;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_DRIVERS_MCP2515_H_
