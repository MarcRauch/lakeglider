#include "hw/interfaces/mock/AdcMock.hh"

namespace gl::hw {
double AdcMock::read(PinAnalogSensor pin) const {
  if (!pinValMap.contains(pin)) {
    return -1.;
  }
  return pinValMap.at(pin);
}

void AdcMock::setValue(PinAnalogSensor pin, double val) {
  pinValMap[pin] = val;
}
}  // namespace gl::hw
