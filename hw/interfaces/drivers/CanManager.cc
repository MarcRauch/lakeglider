#include <algorithm>
#include <cstring>
#include <iterator>

#include "hw/interfaces/drivers/CanManager.hh"

namespace gl::hw {

CanManager::CanManager(std::unique_ptr<Mcp2515> mcp2515) : mcp2515(std::move(mcp2515)) {}

void CanManager::loop() {
  // Receive
  CanFrame recvFrame;
  CanId canId;
  std::tie(recvFrame.len, canId) = mcp2515->read(recvFrame.data);
  while (recvFrame.len > 0) {
    auto bufferIt = std::ranges::find_if(recvBuffers, [&](const RecvBuffer& buff) {
      return buff.isValid && buff.canId == canId && buff.msgId == recvFrame.msgId();
    });
    // No entry yet, find empty candidate
    if (bufferIt == recvBuffers.end()) {
      bufferIt = std::ranges::find_if(recvBuffers, [&](const RecvBuffer& buff) { return !buff.isValid; });
      // No empty entry, delete oldest candidate
      if (bufferIt == recvBuffers.end()) {
        // Check for overflow
        bufferIt =
            std::ranges::find_if(recvBuffers, [&](const RecvBuffer& buff) { return buff.lastUsed > loopCounter; });
        if (bufferIt == recvBuffers.end()) {
          bufferIt = std::ranges::min_element(recvBuffers, {}, &RecvBuffer::lastUsed);
        }
      }
      bufferIt->isValid = true;
      bufferIt->canId = canId;
      bufferIt->msgId = recvFrame.msgId();
      bufferIt->numRecv = 0;
      bufferIt->msgLen = -1;
    }

    if (recvFrame.isMsgStart()) {
      if (bufferIt->msgLen > 0) {
        bufferIt->isValid = false;
        std::tie(recvFrame.len, canId) = mcp2515->read(recvFrame.data);
        continue;
      }
      bufferIt->msgLen = static_cast<uint8_t>(recvFrame.data[1]);
    }

    if (bufferIt->msgLen > 0 && bufferIt->numRecv >= bufferIt->msgLen) {
      bufferIt->isValid = false;
    } else {
      bufferIt->buffer[bufferIt->numRecv] = recvFrame;
      bufferIt->numRecv++;
    }
    std::tie(recvFrame.len, canId) = mcp2515->read(recvFrame.data);
  }

  //Send TODO: This is very hacky. we need to avoid the receive buffer overflowing, so we only send one buffer max and only
  // send once every 5 cycles. Possible fixes
  // - CAN Fd with larger buffers
  // - Add flow control
  // - Add interrupt line on new message
  if (loopCounter % 5 == 0 && sendHead != sendTail) {
    if (mcp2515->send(sendBuffer[sendTail].data, sendBuffer[sendTail].len)) {
      sendTail = (sendTail + 1) % sendBuffer.size();
    }
  }
  loopCounter++;
}

std::optional<CanManager::CanMsg> CanManager::readMsg() {
  auto bufferIt = std::ranges::find_if(
      recvBuffers, [&](const RecvBuffer& buff) { return buff.isValid && buff.numRecv == buff.msgLen; });
  if (bufferIt == recvBuffers.end() || bufferIt->msgLen <= 0) {
    return std::nullopt;
  }
  std::span<CanFrame> frames = std::span<CanFrame>(bufferIt->buffer).first(bufferIt->msgLen);

  std::ranges::sort(frames, [](const CanFrame& f1, const CanFrame& f2) { return f1.seqNr() < f2.seqNr(); });
  CanMsg result{.canId = bufferIt->canId, .msgLen = 0};
  for (const CanFrame& frame : frames) {
    const uint8_t startIdx = frame.isMsgStart() ? 2 : 1;
    for (uint32_t i = startIdx; i < frame.len; i++) {
      result.data[result.msgLen] = frame.data[i];
      result.msgLen++;
    }
  }
  bufferIt->isValid = false;
  return result;
}

bool CanManager::writeMsg(std::span<const std::byte> data) {
  if (data.size() == 0) {
    return false;
  }
  const uint8_t numFrames = 1 + data.size() / 7;
  const uint32_t framesAvailable = sendBuffer.size() - ((sendHead + sendBuffer.size() - sendTail) % sendBuffer.size());
  if (framesAvailable < numFrames) {
    return false;
  }

  const auto appendToBuffer = [&](CanFrame&& frame) {
    sendBuffer[sendHead] = std::move(frame);
    sendHead = (sendHead + 1) % sendBuffer.size();
  };

  std::array<std::byte, 8> initData;
  initData[0] = static_cast<std::byte>(0x80 | (msgIdCounter << 5));
  initData[1] = static_cast<std::byte>(numFrames);
  const uint8_t initLen = std::min<uint8_t>(6u, data.size());
  auto dataIt = data.begin();
  std::copy_n(dataIt, initLen, std::next(initData.begin(), 2));
  appendToBuffer(CanFrame{.len = static_cast<uint8_t>(initLen + 2), .data = std::move(initData)});
  std::advance(dataIt, initLen);

  for (uint8_t i = 1; i < numFrames; i++) {
    const uint8_t frameLen = std::min<uint32_t>(7u, std::distance(dataIt, data.end()));
    std::array<std::byte, 8> frameData;
    frameData[0] = static_cast<std::byte>((msgIdCounter << 5) | i);
    std::copy_n(dataIt, frameLen, std::next(frameData.begin(), 1));
    appendToBuffer(CanFrame{.len = static_cast<uint8_t>(frameLen + 1), .data = std::move(frameData)});
    std::advance(dataIt, frameLen);
  }

  msgIdCounter = (msgIdCounter + 1) % 4;
  return true;
}

}  // namespace gl::hw
