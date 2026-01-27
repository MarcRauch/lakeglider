#ifndef GL_COM_MSG_TYPE_H_
#define GL_COM_MSG_TYPE_H_

#include <stdint.h>

namespace gl::msg {
/**
Message types
*/
enum class Type : uint8_t {
  Dummy = 0x00,
  Battery = 0x01,
  Depth = 0x02,
  Imu = 0x03,
  Time = 0x04,
  Subscriptions = 0x05,
  Compass = 0x06
};

}  // namespace gl::msg

#endif  // GL_COM_MSG_TYPE_H_
