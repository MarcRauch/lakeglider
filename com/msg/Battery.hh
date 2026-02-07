#ifndef GL_COM_MSG_BATTERY_H_
#define GL_COM_MSG_BATTERY_H_

#include "com/msg/Type.hh"

namespace gl::msg {

/**
Message for battery reading
*/
struct __attribute__((packed)) Battery {
  static constexpr Type TYPE = Type::Battery;
  float voltage_v;
  float cellVoltage_v;
  float percentage;
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_BATTERY_H_
