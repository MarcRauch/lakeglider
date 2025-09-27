#include "hw/sensors/Potentiometer.hh"

namespace gl::hw {

Potentiometer::Potentiometer(IAdc* adc, double endLow_v, double endHigh_v)
    : adc(adc), endLow_v(endLow_v), endHigh_v(endHigh_v) {}

double Potentiometer::read() {
  const double currentValue_v = adc->read();
  return (currentValue_v - endLow_v) / (endHigh_v - endLow_v);
}
}  // namespace gl::hw