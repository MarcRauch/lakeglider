#include "msgs/MsgHandler.hh"

#include <pico/stdlib.h>
#include <stdio.h>

namespace {
bool sendByte(uint8_t data, bool isEnd) {
  bool success = true;
  // Byte stuffing: Escape special bytes if the message is not finished.
  if (!isEnd && (data == gl::msg::MsgHandler::ESCAPE || data == gl::msg::MsgHandler::MSGEND)) {
    success &= putchar_raw(gl::msg::MsgHandler::ESCAPE) >= 0;
  }
  success &= putchar_raw(data) >= 0;
  return success;
}
}  // namespace

namespace gl {
namespace msg {
MsgHandler::MsgHandler(uint8_t* payloadBuffer) : payload(payloadBuffer) {
  stdio_init_all();
  resetMsg();
}

bool MsgHandler::send() {
  bool success = true;

  // Send header
  uint8_t* headerPtr = (uint8_t*)&header;
  for (uint8_t i = 0; i < sizeof(MsgHeader); i++) {
    success &= sendByte(headerPtr[i], false);
    checksumCounter += headerPtr[i];
  }

  // Send payload
  for (uint8_t i = 0; i < header.msgLength; i++) {
    success &= sendByte(payload[i], false);
    checksumCounter += payload[i];
  }

  // Finish
  checksum = calculateCheckSum();
  uint8_t* checksumPtr = (uint8_t*)&checksum;
  success &= sendByte(checksumPtr[0], false) & sendByte(checksumPtr[1], false);
  success &= sendByte(MSGEND, true);
  return success;
}

bool MsgHandler::addByte(const uint8_t& data) {
  if (isFinalized) {
    resetMsg();
    isFinalized = false;
  }

  bool isNextEscaped = false;
  if (data == ESCAPE && !isEscaped) {
    isNextEscaped = true;
    return false;
  }

  // Finalize message
  if (data == MSGEND && !isEscaped) {
    isFinalized = true;
    if (checksum == calculateCheckSum()) {
      return true;
    }
    return false;
  }

  // Save data to correct location.
  uint8_t* dataPtr;
  uint16_t dataOffset;
  if (readCounter < sizeof(MsgHeader)) {
    dataPtr = (uint8_t*)&header;
    dataOffset = readCounter;
    checksumCounter += data;
  } else if (readCounter < sizeof(MsgHeader) + header.msgLength) {
    dataPtr = payload;
    dataOffset = readCounter - sizeof(MsgHeader);
    checksumCounter += data;
  } else {
    dataPtr = (uint8_t*)&checksum;
    dataOffset = readCounter - sizeof(MsgHeader) - header.msgLength;
  }
  dataPtr[dataOffset] = data;
  readCounter++;
  isEscaped = isNextEscaped;
  return false;
}

void MsgHandler::resetMsg() {
  isFinalized = false;
  isEscaped = false;
  checksum = 0;
  readCounter = 0;
  checksumCounter = 0;
}

uint16_t MsgHandler::calculateCheckSum() {
  // See https://barrgroup.com/blog/crc-series-part-1-additive-checksums
  uint32_t temp = (checksumCounter >> 16) + (checksumCounter & 0xFFFF);
  temp += temp >> 16;
  return ((uint16_t)~temp);
}

}  // namespace msg
}  // namespace gl
