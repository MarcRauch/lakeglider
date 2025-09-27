#include "hw/actuators/Valve.hh"

namespace gl::hw {
Valve::Valve(IGpio* gpio) : gpio(gpio) {
  close();
}

void Valve::open() {
  gpio->setHigh();
}

void Valve::close() {
  gpio->setLow();
}

}  // namespace gl::hw
