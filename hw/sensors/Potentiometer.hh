#ifndef GL_HW_ACTUATORS_POTENTIOMETER_H_
#define GL_HW_ACTUATORS_POTENTIOMETER_H_

#include "hw/Pins.hh"
#include "hw/interfaces/IAdc.hh"

namespace gl::hw {

class Potentiometer {
 public:
  /**
   * Create a potentiometer instance at a certain analog pin
   * @param[in] adc Adc connected to desired pin
   * @param[in] endLow_v The voltage at the position giving the lowest voltage
   * @param[in] endRight_v The voltage at the position giving the highest voltage
   * @returns Potentiometer object
   */
  Potentiometer(IAdc* adc, double endLow_v, double endHigh_v);

  /**
   * Read the current position
   * @returns The current position in the range [0,1]
   */
  double read();

 private:
  IAdc* adc;
  const double endLow_v;
  const double endHigh_v;
};

}  // namespace gl::hw

#endif  // GL_HW_ACTUATORS_POTENTIOMETER_H_