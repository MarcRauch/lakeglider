#include "hw/interfaces/pico/AdcPico.hh"

namespace gl::hw {

double AdcPico::read() const {
  adc_select_input(pinNr);
  return 3.3 * static_cast<double>(adc_read()) / 4095.;
}

}  // namespace gl::hw