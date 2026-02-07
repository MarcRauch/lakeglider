#ifndef GL_HW_INTERFACES_DRIVERS_CANMANAGER_H_
#define GL_HW_INTERFACES_DRIVERS_CANMANAGER_H_

#include <array>
#include <memory>
#include <optional>

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
    static constexpr uint32_t MAX_MSG_LEN = 256;
    CanId canId;
    uint32_t msgLen;
    std::array<std::byte, MAX_MSG_LEN> data;
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

  struct RecvBuffer {
    static constexpr uint32_t MAX_FRAMES = 64;
    bool isValid = false;
    CanId canId;
    uint8_t msgId;
    uint32_t lastUsed;
    int32_t msgLen;
    uint16_t numRecv = 0;
    std::array<CanFrame, MAX_FRAMES> buffer;
  };
  static constexpr uint32_t MAX_PARTIAL_RECV = 8;
  std::array<RecvBuffer, MAX_PARTIAL_RECV> recvBuffers;

  static constexpr uint32_t SEND_QUEUE_SIZE = 512;
  std::array<CanFrame, SEND_QUEUE_SIZE> sendBuffer;
  uint32_t sendHead = 0;
  uint32_t sendTail = 0;

  uint8_t msgIdCounter = 0;
  uint32_t loopCounter = 0;
};
}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_DRIVERS_CANMANAGER_H_
