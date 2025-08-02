#ifndef GL_COM_MSG_HEADER_H_
#define GL_COM_MSG_HEADER_H_

#include <stdint.h>

#include "com/msg/Type.hh"

namespace gl::msg {

/**
Header containing message metadata
See details on https://docs.google.com/document/d/1rO527Eti_Bo6kDqbO_FjO-jc7IsgAhpVrAcGxc96Hw4/edit
*/
struct __attribute__((packed)) Header {
  uint8_t version = 0x01;
  Type type;
  uint8_t deviceId;
  uint8_t msgLength;
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_HEADER_H_
