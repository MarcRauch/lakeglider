#ifndef GL_COM_MSG_HANDLER_H_
#define GL_COM_MSG_HANDLER_H_

#include <stdint.h>

#include "msgs/MsgHeader.hh"
#include "msgs/MsgType.hh"
#include "utils/GlTime.hh"

namespace gl {
namespace msg {

/**
Base message class used to serialize and deserialize messages.
See details on
https://docs.google.com/document/d/1rO527Eti_Bo6kDqbO_FjO-jc7IsgAhpVrAcGxc96Hw4/edit
*/
class MsgHandler {
 public:
  /**
   * Constructs a MsgHandler object. This takes a message and prepares it to be
   * sent.
   * @param[in] msg The message to be sent.
   * @param[in] deviceId The id of the device that produced the message.
   * @param[in] timestamp The timestamp (since system start) in us.
   * @returns Corresponding MsgHandler object
   */
  template <typename T>
  MsgHandler(const T& msg, const uint8_t& deviceId, const time::GlTime timestamp) {
    header.timestamp_us = timestamp.usec<uint64_t>();
    header.deviceId = deviceId;
    header.type = msg.TYPE;
    header.msgLength = sizeof(T);
    payload = (uint8_t*)&msg;
  }
  /**
   * Constructs a MsgHandler object. This takes an empty buffer preparing to
   * receive a message.
   * @param[in] payloadBuffer A buffer of the length of the longest message. Is
   * used to buffer received data.
   * @returns Corresponding MsgHandler object
   */
  MsgHandler(uint8_t* payloadBuffer);

  /**
   * After the message payload buffer has been filled up, the message can be
   * deserialized.
   * @param[in] msg A pointer to the start of the message object to be filled.
   * @returns True if the deserialization was successful.
   */
  template <typename T>
  bool deserialize(T* msg) const {
    if (header.msgLength != sizeof(T)) {
      return false;
    }
    uint8_t* msgPtr = (uint8_t*)msg;
    for (uint8_t i = 0; i < sizeof(T); i++) {
      msgPtr[i] = payload[i];
    }
    return true;
  }
  /**
   * Sends the full message over the UART interface.
   * @returns True if the message was sent successfuly.
   */
  bool send();
  /**
   * Takes a received byte and saves it at the corresponding location.
   * @param[in] data The data byte to be added to the received message.
   * @returns True if a valid message has been completly received.
   */
  bool addByte(const uint8_t& data);

  /**
   * Gets the timestamp in ms associated with this message
   * @returns The corresponding unix timestamp.
   */
  time::GlTime getTimestamp() const { return time::GlTime::usec(header.timestamp_us); }
  /**
   * Gets the device ID that produced this message.
   * @returns The corresponding device ID.
   */
  uint8_t getDeviceId() const { return header.deviceId; }

  // Consts
  static constexpr uint8_t MSGEND = 0x22;
  static constexpr uint8_t ESCAPE = 0x73;

 private:
  /**
   * Resets the received message. Either to receive the next message or to
   * discard a faulty message.
   */
  void resetMsg();
  /**
   * Calculates the checksum given on the current checksumCounter. The counter
   * needs to be updated before.
   * @returns The 2 Byte checksum.
   */
  uint16_t calculateCheckSum();

  // Message Header
  MsgHeader header;

  // Payload
  uint8_t* payload;

  // Footer
  uint16_t checksum;

  // Helpers
  bool isEscaped = false;
  bool isFinalized = false;
  uint16_t readCounter = 0;
  uint32_t checksumCounter = 0;
};

}  // namespace msg
}  // namespace gl

#endif  // GL_COM_MSG_HANDLER_H_
