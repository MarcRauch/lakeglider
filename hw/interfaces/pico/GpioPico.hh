#ifndef GL_HW_INTERFACES_PICO_GPIOPICO_H_
#define GL_HW_INTERFACES_PICO_GPIOPICO_H_

#include <hardware/gpio.h>
#include <stdint.h>

#include "hw/Pins.hh"
#include "hw/interfaces/IGpio.hh"

namespace gl::hw {

/**
Implements the GPIO interface for the rp2350
*/
class GpioPico : public IGpio {
 public:
  /**
   * Constructs a GPIO object and initializes it
   * @param[in] pin The pin to be controlled
   * @param[in] isOutput True if output, false if input
   * @returns GPIO object
   */
  template <ConceptPinGpio T>
  GpioPico(T pin, bool isOutput) : pinNr(static_cast<uint8_t>(pin)) {
    gpio_init();
    gpio_init(pinNr);
    gpio_set_dir(pinNr, isOutput);
  }

  void setHigh() override;

  void setLow() override;

 private:
  const uint8_t pinNr;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_GPIOPICO_H_
