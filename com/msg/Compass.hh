#ifndef GL_COM_MSG_COMPASS_H_
#define GL_COM_MSG_COMPASS_H_

#include "com/msg/Type.hh"

namespace gl::msg {
/**
Message for Compass reading.
*/
struct __attribute__((packed)) Compass {
  static constexpr Type TYPE = Type::Compass;
  float mag_ut[3];
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_COMPASS_H_
