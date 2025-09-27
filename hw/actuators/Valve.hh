#ifndef GL_HW_ACTUATORS_VALVE_H_
#define GL_HW_ACTUATORS_VALVE_H_

#include "hw/interfaces/IGpio.hh"

namespace gl::hw {
class Valve {
 public:
  /**
   * Create a valve object
   * @param[in] gpioValve Gpio pin connected to the valve
   * @returns Valve object
   */
  Valve(IGpio* gpio);

  /**
   * Opens the valve
   */
  void open();

  /**
   * Closes the valve
   */
  void close();

 private:
  IGpio* gpio;
};

}  // namespace gl::hw

#endif  // GL_HW_ACTUATORS_VALVE_H_