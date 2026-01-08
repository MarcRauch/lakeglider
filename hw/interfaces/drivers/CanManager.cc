#include <algorithm>
#include <cstring>
#include <iterator>

#include "hw/interfaces/drivers/CanManager.hh"

namespace {
constexpr uint32_t MAX_RECV_BUFFER_LEN = 32;
constexpr uint32_t MAX_SEND_QUEUE_LEN = 256;
}  // namespace

namespace gl::hw {

CanManager::CanManager(std::unique_ptr<Mcp2515> mcp2515) : mcp2515(std::move(mcp2515)) {}

void CanManager::loop() {
  // Receive
  CanFrame recvFrame;
  CanId canId;
  std::tie(recvFrame.len, canId) = mcp2515->read(recvFrame.data);
  while (recvFrame.len > 0) {
    // Clean up
    while (recvBuffer.size() > MAX_RECV_BUFFER_LEN) {
      // This should not happen. TODO: Do not delete random entry
      recvBuffer.erase(recvBuffer.begin());
    }

    // Buffer frame
    const BufferKey currKey{canId, recvFrame.msgId()};
    if (!recvBuffer.contains(currKey)) {
      recvBuffer[currKey] = CanBuffer{};
    }
    CanBuffer& currBuff = recvBuffer[currKey];
    if (currBuff.data.size() >= currBuff.len) {
      currBuff = CanBuffer{};
    }
    if (currBuff.data.size() == 0) {
      if (!recvFrame.isMsgStart()) {
        std::tie(recvFrame.len, canId) = mcp2515->read(recvFrame.data);
        continue;
      }
      currBuff.len = static_cast<uint32_t>(recvFrame.data[1]);
    }
    currBuff.data.push_back(recvFrame);
    std::tie(recvFrame.len, canId) = mcp2515->read(recvFrame.data);
  }

  //Send TODO: This is very hacky. we need to avoid the receive buffer overflowing, so we only send one buffer max and only
  // send once every 5 cycles. Fix this by connecting the receive buffer interrupt lines.
  if (sendBuffer.size() > 0 && loopCounter % 5 == 0 && mcp2515->send(sendBuffer.front().data, sendBuffer.front().len)) {
    sendBuffer.pop();
  }
  loopCounter++;
}

std::optional<CanManager::CanMsg> CanManager::readMsg() {
  auto fullBuff = std::ranges::find_if(recvBuffer, [](const auto& currBuff) {
    return currBuff.second.data.size() == currBuff.second.len && currBuff.second.len > 0;
  });
  if (fullBuff == recvBuffer.end()) {
    return std::nullopt;
  }
  auto& [fullKey, fullFrames] = *fullBuff;

  std::ranges::sort(fullFrames.data, [](const CanFrame& f1, const CanFrame& f2) { return f1.seqNr() < f2.seqNr(); });
  CanMsg result{.canId = fullKey.first};
  for (const CanFrame& frame : fullFrames.data) {
    const uint8_t startIdx = frame.isMsgStart() ? 2 : 1;
    for (uint32_t i = startIdx; i < frame.len; i++) {
      result.data.push_back(frame.data[i]);
    }
  }
  recvBuffer.erase(fullKey);
  return result;
}

bool CanManager::writeMsg(std::span<const std::byte> data) {
  if (data.size() == 0) {
    return false;
  }
  const uint8_t numFrames = 1 + data.size() / 7;

  std::array<std::byte, 8> initData;
  initData[0] = static_cast<std::byte>(0x80 | (msgIdCounter << 5));
  initData[1] = static_cast<std::byte>(numFrames);
  const uint8_t initLen = std::min<uint8_t>(6u, data.size());
  auto dataIt = data.begin();
  std::copy_n(dataIt, initLen, std::next(initData.begin(), 2));
  sendBuffer.push(CanFrame{.len = static_cast<uint8_t>(initLen + 2), .data = std::move(initData)});
  std::advance(dataIt, initLen);

  for (uint8_t i = 1; i < numFrames; i++) {
    const uint8_t frameLen = std::min<uint32_t>(7u, std::distance(dataIt, data.end()));
    std::array<std::byte, 8> frameData;
    frameData[0] = static_cast<std::byte>((msgIdCounter << 5) | i);
    std::copy_n(dataIt, frameLen, std::next(frameData.begin(), 1));
    sendBuffer.push(CanFrame{.len = static_cast<uint8_t>(frameLen + 1), .data = std::move(frameData)});
    std::advance(dataIt, frameLen);
  }

  while (sendBuffer.size() > MAX_SEND_QUEUE_LEN) {
    sendBuffer.pop();
  }
  msgIdCounter = (msgIdCounter + 1) % 4;
  return true;
}

}  // namespace gl::hw
