#ifndef GL_COM_MSG_COMPASS_H_
#define GL_COM_MSG_COMPASS_H_

#include <stdint.h>

#include "com/msg/Type.hh"

namespace gl {
namespace msg {

/**
Message for Compass reading.
*/
struct __attribute__((packed)) CompassMsg {
  static constexpr Type TYPE = Type::Compass;
  float mag_ut[3];
};
}  // namespace msg
}  // namespace gl

#endif  // GL_COM_MSG_COMPASS_H_
