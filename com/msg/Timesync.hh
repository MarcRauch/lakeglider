#ifndef GL_COM_MSG_TIMESYNC_H_
#define GL_COM_MSG_TIMESYNC_H_

#include "com/msg/Type.hh"
#include "hw/Pins.hh"

namespace gl::msg {

/**
Message for timestamp synchronization.
*/
struct __attribute__((packed)) TimeSync {
  static constexpr Type TYPE = Type::TimeSync;
  hw::CanId canId;
  uint64_t initialTimestamp_us;
  uint64_t recvTimestamp_us;
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_TIMESYNC_H_
