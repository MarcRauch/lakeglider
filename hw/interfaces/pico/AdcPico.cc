#include "hw/interfaces/pico/AdcPico.hh"

#include <hardware/adc.h>

namespace gl::hw {
AdcPico::AdcPico() {
  adc_init();
}

double AdcPico::read(PinAnalogSensor pin) const {
  adc_select_input(static_cast<uint8_t>(pin));
  return static_cast<double>(adc_read()) / 4095.;
}

}  // namespace gl::hw