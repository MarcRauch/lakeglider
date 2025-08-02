#ifndef GL_HW_INTERFACES_PICO_ADCPICO_H_
#define GL_HW_INTERFACES_PICO_ADCPICO_H_

#include <stdint.h>

#include "hw/interfaces/IAdc.hh"

namespace gl::hw {

/**
Implements the ADC interface for the rp2350
*/
class AdcPico : public IAdc {
 public:
  /**
   * Initializes the ADC
   **/
  AdcPico();

  double read(PinAnalogSensor pin) const override;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_ADCPICO_H_
