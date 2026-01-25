#ifndef GL_COM_MSG_SUBSCRIPTIONS_H_
#define GL_COM_MSG_SUBSCRIPTIONS_H_

#include <stdint.h>

#include "com/msg/Type.hh"

namespace gl::msg {

/**
Message for subscribing to message types.
*/
struct __attribute__((packed)) Subscriptions {
  static constexpr Type TYPE = Type::Subscriptions;
  char name[16];
  uint8_t nSubscriptions;
  Type subscriptions[64];
};
}  // namespace gl::msg

#endif  // GL_COM_MSG_SUBSCRIPTIONS_H_
