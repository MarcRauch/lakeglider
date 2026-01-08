#ifndef GL_HW_INTERFACES_DRIVERS_CANMANAGER_H_
#define GL_HW_INTERFACES_DRIVERS_CANMANAGER_H_

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "hw/interfaces/drivers/Mcp2515.hh"

namespace gl::hw {
class CanManager {
 public:
  /**
   * Creates a new CanManager object. The manager converts messages to CAN frames and decodes frames into messages.
   * The framing is done with:
   *
   * First Message:
   * 1, msgId (2 Bits), SeqNr(5 Bits) | MsgLen(1 Byte) | data ...
   *
   * Other messages:
   * 0, msgId (2 Bits), SeqNr(5 Bits) | data ...
   *
   */
  CanManager(std::unique_ptr<Mcp2515> mcp2515);

  /**
   * Containing the sending canId and the received data
   */
  struct CanMsg {
    CanId canId;
    std::vector<std::byte> data;
  };

  /**
   * This needs to be run very quickly to not miss frames
   */
  void loop();

  /**
  *  Reads fully received messages
  * @returns CanMsg if a new message has been completed. std::nullopt otherwise.
  */
  std::optional<CanMsg> readMsg();

  /**
   * submits message for sending
   * @param[in] data bytes to be sent
   * @returns True if the queue is not full yet.
   */
  bool writeMsg(std::span<const std::byte> data);

 private:
  std::unique_ptr<Mcp2515> mcp2515;

  struct CanFrame {
    uint8_t seqNr() const { return static_cast<uint8_t>(data[0]) & 0x1F; };
    uint8_t msgId() const { return (static_cast<uint8_t>(data[0]) & 0x60) >> 5; };
    bool isMsgStart() const { return static_cast<uint8_t>(data[0]) & 0x80; };
    uint8_t len;
    std::array<std::byte, 8> data;
  };

  struct CanBuffer {
    uint16_t len = 0;
    std::vector<CanFrame> data;
  };
  // Key: canId, msgId
  using BufferKey = std::pair<CanId, uint8_t>;
  std::map<BufferKey, CanBuffer> recvBuffer;

  std::queue<CanFrame> sendBuffer;
  uint8_t msgIdCounter = 0;

  uint32_t loopCounter = 0;
};
}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_DRIVERS_CANMANAGER_H_
