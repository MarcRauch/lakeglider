#ifndef GL_HW_INTERFACES_MOCK_ADCMOCK_H_
#define GL_HW_INTERFACES_MOCK_ADCMOCK_H_

#include <stdint.h>
#include <unordered_map>

#include "hw/interfaces/IAdc.hh"

namespace gl::hw {

/**
Implements a mock ADC interface for testing.
*/
class AdcMock : public IAdc {
 public:
  double read(PinAnalogSensor pin) const override;

  /**
   * Set the value which will be read after (for testing purpose)
   * @param[in] pin Pin number to set.
   * @param[in] val Value to set that pin to.
   */
  void setValue(PinAnalogSensor pin, double val);

 private:
  std::unordered_map<PinAnalogSensor, double> pinValMap;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_MOCK_ADCMOCK_H_
