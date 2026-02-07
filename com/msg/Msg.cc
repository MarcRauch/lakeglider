#include "com/msg/Msg.hh"

namespace {
uint16_t calculateCrcImpl(std::span<const std::byte> buffer) {
  uint32_t checksumCounter = 0;
  for (const std::byte& b : buffer) {
    checksumCounter += static_cast<uint32_t>(b);
  }
  checksumCounter = (checksumCounter >> 16) + (checksumCounter & 0xFFFF);
  const uint16_t checksumInv = static_cast<uint16_t>(checksumCounter + (checksumCounter >> 16));
  return ~checksumInv;
}

uint16_t getCrcImpl(std::span<const std::byte> buffer) {
  uint16_t checksum;
  std::memcpy(&checksum, buffer.last(2).data(), 2);
  return checksum;
}
}  // namespace

namespace gl::msg {

std::optional<Msg> Msg::fromBytes(std::span<const std::byte> data) {
  if (data.size() < sizeof(gl::msg::Header) + 2) {
    return std::nullopt;
  }
  const uint16_t checksumCalc = calculateCrcImpl(data.first(data.size() - 2));
  const uint16_t checksumRecv = getCrcImpl(data);
  if (checksumCalc != checksumRecv) {
    return std::nullopt;
  }
  Msg msg;
  msg.msgLen = data.size();
  std::memcpy(&msg.header, data.data(), sizeof(header));
  std::copy(data.begin(), data.end(), msg.serializedMsg.begin());
  return msg;
}

std::span<const std::byte> Msg::getSerializedMsg() const {
  return std::span(serializedMsg).first(msgLen);
}

gl::msg::Header Msg::getHeader() const {
  return header;
}

uint16_t Msg::calculateCrc() const {
  return calculateCrcImpl(getSerializedMsg().first(msgLen - 2));
}

uint16_t Msg::getCrc() const {
  return getCrcImpl(getSerializedMsg());
}
}  // namespace gl::msg
