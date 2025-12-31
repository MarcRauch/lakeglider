#ifndef GL_HW_INTERFACES_PICO_ADCPICO_H_
#define GL_HW_INTERFACES_PICO_ADCPICO_H_

#include <hardware/adc.h>
#include <stdint.h>

#include "hw/Pins.hh"
#include "hw/interfaces/IAdc.hh"

namespace gl::hw {

/**
Implements the ADC interface for the rp2350
*/
class AdcPico : public IAdc {
 public:
  /**
   * Initializes the ADC
   * @param[in] pin The pin to be read
   * @returns ADC object
   **/
  template <ConceptPinAnalog T>
  AdcPico(T pin) : pinNr(static_cast<uint8_t>(pin)) {
    adc_init();
  };

  double read() const override;

 private:
  const uint8_t pinNr;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_ADCPICO_H_
