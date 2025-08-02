#ifndef GL_HW_INTERFACES_IADC_H_
#define GL_HW_INTERFACES_IADC_H_

#include <stdint.h>

#include "hw/Pins.hh"

namespace gl::hw {

/**
Interface for analog reading functionality.
*/
class IAdc {
 public:
  virtual ~IAdc() = default;

  /**
   * Read the current ADC value on the specified pin.
   * @param[in] pin Pin number to read.
   * @returns The read value normalized to [0,1]
   */
  virtual double read(PinAnalogSensor pinNr) const = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_IADC_H_
