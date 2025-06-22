#ifndef GL_COM_MSG_BATTERY_H_
#define GL_COM_MSG_BATTERY_H_

#include <stdint.h>

#include "com/msg/Type.hh"

namespace gl {
namespace msg {

/**
Message for battery reading
*/
struct __attribute__((packed)) Battery {
  static constexpr Type TYPE = Type::Battery;
  float voltage_v;
  float cellVoltage_v;
  float percentage;
};
}  // namespace msg
}  // namespace gl

#endif  // GL_COM_MSG_BATTERY_H_
