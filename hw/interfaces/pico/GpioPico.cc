#include "hw/interfaces/pico/GpioPico.hh"

namespace gl::hw {
void GpioPico::setHigh() {
  gpio_put(pinNr, true);
}

void GpioPico::setLow() {
  gpio_put(pinNr, false);
}
}  // namespace gl::hw
