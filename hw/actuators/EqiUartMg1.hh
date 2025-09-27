#ifndef GL_HW_ACTUATORS_EQIUARTMG1_H_
#define GL_HW_ACTUATORS_EQIUARTMG1_H_

#include <stdint.h>
#include <string>

#include "hw/interfaces/IUart.hh"

namespace gl::hw {

/**
Pump controller. Based on a EQi-UART-MG1 based pump see
https://micropumps.co.uk/wp-content/uploads/2022/09/IM220-TCS-EQI-UART-User-Guide-REV-2.pdf
*/
class EqiUartMg1 {
 public:
  /**
   * Create a pump object
   * @param[in] uart Uart port connected to the pump
   * @returns Pump object
   */
  EqiUartMg1(IUart* uart);

  /**
   * Start the pump
   */
  void start();

  /**
   * Stop the pump
   */
  void stop();

  /**
   * Set the flow direction 
   * @param[in] pumpIn Direction to pump
   */
  void setDirection(bool pumpIn);

  /**
   * Set the motor speed
   * @param[in] speed The speed in the range [0,1]
   */
  void setSpeed(double speed);

 private:
  IUart* uart;
};

}  // namespace gl::hw

#endif  // GL_HW_ACTUATORS_EQIUARTMG1_H_