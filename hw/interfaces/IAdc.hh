#ifndef GL_HW_INTERFACES_IADC_H_
#define GL_HW_INTERFACES_IADC_H_

#include <stdint.h>

#include "hw/Pins.hh"

namespace gl::hw {

/**
Interface for analog reading functionality. Assumes instance for each pin.
*/
class IAdc {
 public:
  virtual ~IAdc() = default;

  /**
   * Read the current ADC value on the specified pin.
   * @returns The read value in volt.
   */
  virtual double read() const = 0;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_IADC_H_
