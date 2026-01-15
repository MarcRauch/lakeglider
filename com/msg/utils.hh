
#ifndef GL_COM_MSG_UTILS_H_
#define GL_COM_MSG_UTILS_H_

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

uint16_t calculateCrc(std::span<std::byte> buffer) {
  uint32_t checksumCounter = 0;
  for (const std::byte& b : buffer) {
    checksumCounter += static_cast<uint32_t>(b);
  }
  checksumCounter = (checksumCounter >> 16) + (checksumCounter & 0xFFFF);
  const uint16_t checksumInv = static_cast<uint16_t>(checksumCounter + (checksumCounter >> 16));
  return ~checksumInv;
}

}  // namespace

namespace gl::msg {

/**
 * Serializes a message into a vector. Creates and appends a header
 * @params[in] msg The messge to be serialized
 * @params[in] deviceId The id of the device that produced the message
 * @params[in] timestamp The timestamp at which the message was generated
 * @returns a vector containing the serialized message
 */
std::vector<std::byte> serialize(IsMsg auto msg, uint8_t deviceId, gl::utils::Time timestamp) {
  const gl::msg::Header header = {
      .type = msg.TYPE, .timestamp_us = timestamp.usec<uint64_t>(), .deviceId = deviceId, .msgLength = sizeof(msg)};

  std::vector<std::byte> buffer;
  const auto appendToBuffer = [&buffer](const auto& data) {
    std::span<const std::byte> bytes = std::as_bytes(std::span(&data, 1));
    return buffer.insert(buffer.end(), bytes.begin(), bytes.end());
  };
  appendToBuffer(header);
  appendToBuffer(msg);

  const uint16_t checksum = calculateCrc(buffer);
  appendToBuffer(checksum);

  return buffer;
};

/**
 * Deserializes the header of a message. Also checks the checksum.
 * @params[in] buffer Span containing the received message bytes
 * @returns nullopt if the deserialization fails, the header otherwise
 */
std::optional<gl::msg::Header> deserializeHeader(const std::span<std::byte> buffer) {
  if (buffer.size() < sizeof(gl::msg::Header) + 2) {
    return std::nullopt;
  }
  const uint16_t checksumCalc = calculateCrc(buffer.first(buffer.size() - 2));
  uint16_t checksumRecv;
  std::memcpy(&checksumRecv, buffer.last(2).data(), 2);
  if (checksumCalc != checksumRecv) {
    return std::nullopt;
  }

  gl::msg::Header header;
  std::memcpy(&header, buffer.data(), sizeof(header));
  return header;
}

/**
 * Deserializes a message. Needed to have decoded the header in advance.
 * @params[in] buffer Span containing the received message bytes
 * @params[in] header The previously decoded header
 * @returns nullopt if the deserialization fails, the message otherwise
 */
template <IsMsg Msg>
std::optional<Msg> deserializeMsg(const std::span<std::byte> buffer, const gl::msg::Header& header) {
  if (buffer.size() != sizeof(header) + header.msgLength + 2) {
    return std::nullopt;
  }
  const std::span<std::byte> msgBytes = buffer.subspan(sizeof(header), header.msgLength);
  Msg msg;
  std::memcpy(&msg, msgBytes.data(), header.msgLength);
  return msg;
}

}  // namespace gl::msg

#endif  // GL_COM_MSG_UTILS_H_
