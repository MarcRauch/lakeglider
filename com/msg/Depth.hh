#ifndef GL_COM_MSG_DEPTH_H_
#define GL_COM_MSG_DEPTH_H_

#include <stdint.h>

#include "com/msg/Type.hh"

namespace gl::msg {

/**
Message for Depth reading. Also contains water temperature.
*/
struct __attribute__((packed)) Depth {
  static constexpr Type TYPE = Type::Depth;
  uint64_t timestamp_us;
  float depth_m;
  float pressure_pa;
  float temperature_degc;
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_DEPTH_H_
