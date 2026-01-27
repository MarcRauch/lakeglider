#ifndef GL_COM_MSG_IMU_H_
#define GL_COM_MSG_IMU_H_

#include "com/msg/Type.hh"

namespace gl::msg {

/**
Message for IMU reading.
*/
struct __attribute__((packed)) Imu {
  static constexpr Type TYPE = Type::Imu;
  float acc_m_s2[3];
  float w_rad_s[3];
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_IMU_H_
