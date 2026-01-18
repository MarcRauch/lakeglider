#ifndef GL_COM_MSG_IMU_H_
#define GL_COM_MSG_IMU_H_

#include <stdint.h>

#include "com/msg/Type.hh"

namespace gl::msg {

/**
Message for IMU reading.
*/
struct __attribute__((packed)) Imu {
  static constexpr Type TYPE = Type::Imu;
  float acc_mps2[3];
  float w_radps[3];
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_IMU_H_
