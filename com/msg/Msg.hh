#ifndef GL_COM_MSG_MSG_H_
#define GL_COM_MSG_MSG_H_

#include <concepts>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

#include "com/msg/Header.hh"
#include "com/msg/Type.hh"
#include "utils/time/Time.hh"

namespace {

template <typename T>
concept IsMsg = requires(T v) {
  {T::TYPE}->std::same_as<const gl::msg::Type&>;
};

}  // namespace

namespace gl::msg {
/**
 * Base message class used to serialize and deserialize messages.
 * See details on
 * https://docs.google.com/document/d/1rO527Eti_Bo6kDqbO_FjO-jc7IsgAhpVrAcGxc96Hw4/edit
*/
class Msg {
 public:
  /**
   * Create a serialized message object from any message
   * @param[in] msg Message to serialize
   * @param[in] deviceId Id of the device which produced the data
   * @param[in] timestamp Time at which the data has been generated
   * @returns the serialized message
   */
  template <IsMsg T>
  Msg(const T& msg, uint8_t deviceId, gl::utils::Time timestamp) {
    const gl::msg::Header header = {
        .type = msg.TYPE, .timestamp_us = timestamp.usec<uint64_t>(), .deviceId = deviceId, .msgLength = sizeof(msg)};

    const auto appendToBuffer = [this](const auto& data) {
      std::span<const std::byte> bytes = std::as_bytes(std::span(&data, 1));
      return serializedMsg.insert(serializedMsg.end(), bytes.begin(), bytes.end());
    };
    appendToBuffer(header);
    appendToBuffer(msg);

    const uint16_t checksum = calculateCrc();
    appendToBuffer(checksum);
  }

  Msg() = default;

  /**
   * Generates a serialized message from bytes. Deserializes the header and verifies the checksum
   * @param[in] data Bytes of the full serialized message including checksum
   * @returns nullopt on checksum failure or issue on header deserialization. Serialized message otherwise
   */
  static std::optional<Msg> fromBytes(std::span<const std::byte> data);

  /**
   * Get the stored serialized data
   * @returns const reference to the serialized data
   */
  const std::vector<std::byte>& getSerializedMsg() const;

  /**
   * Gets the deserialized header
   * @returns message header
   */
  gl::msg::Header getHeader() const;

  /**
   * Deserializes the full message into the requested type
   * @returns nullopt if deserialization fails, otherwise the deserialized message.
   */
  template <IsMsg T>
  std::optional<T> getMsg() const {
    if (serializedMsg.size() != sizeof(header) + header.msgLength) {
      return std::nullopt;
    }
    if (header.type != T::TYPE) {
      return std::nullopt;
    }
    const std::span<const std::byte> msgBytes = std::span{serializedMsg}.subspan(sizeof(header), header.msgLength);
    T msg;
    std::memcpy(&msg, msgBytes.data(), header.msgLength);
    return msg;
  }

 private:
  uint16_t calculateCrc() const;

  uint16_t getCrc() const;

  gl::msg::Header header;
  std::vector<std::byte> serializedMsg;
};

}  // namespace gl::msg

#endif
