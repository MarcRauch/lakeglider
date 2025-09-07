#ifndef GL_HW_ACTUATORS_PUMP_H_
#define GL_HW_ACTUATORS_PUMP_H_

#include <stdint.h>
#include <string>

#include "hw/Pins.hh"
#include "hw/interfaces/IAdc.hh"
#include "hw/interfaces/IGpio.hh"
#include "hw/interfaces/IUart.hh"

namespace gl::hw {

/**
Pump controller. Based on a EQi-UART-MG1 based pump see
https://micropumps.co.uk/wp-content/uploads/2022/09/IM220-TCS-EQI-UART-User-Guide-REV-2.pdf
*/
class Pump {
 public:
  /**
   * Create a pump object
   * @param[in] uart Uart port connected to the pump
   * @param[in] adcPotentiometer Adc pin connected to the potentiometer
   * @param[in] gpioValve Gpio pin connected to the valve
   * @param[in] refFull_v Reference voltage of the potentiometer where the bottle is full
   * @param[in] refEmpty_v Reference voltage of the potentiometer where the bottle is empty
   * @returns Pump object
   */
  Pump(IUart* uart, IAdc* adcPotentiometer, IGpio* gpioValve, double refFull_v, double refEmpty_v)
      : uart(uart),
        adcPotentiometer(adcPotentiometer),
        gpioValve(gpioValve),
        refFull_v(refFull_v),
        refEmpty_v(refEmpty_v) {
    // Set full speed. TODO: could be adapted when getting closer to full
    const std::string speedCmd = "ZDDDSMDC = 1000\r";
    uart->writeBytes(reinterpret_cast<const uint8_t*>(speedCmd.c_str()), speedCmd.size());
    setPumpDirection(true);
    stopPump();
    closeValve();
  }

  /**
   * Set a target fullness value
   * @param[in] bottleLevel desired fullness percentage
   */
  void pumpTo(double bottleLevel);

  /**
   * Loops to update the pump direction to get to the wanted fullnes. Should be called regularly
   */
  void loop();

 private:
  void setPumpDirection(bool pumpIn);
  void openValve();
  void closeValve();
  void startPump();
  void stopPump();
  double readPotentiometer();

  IUart* uart;
  IAdc* adcPotentiometer;
  IGpio* gpioValve;
  const double refFull_v;
  const double refEmpty_v;
  double targetLevel_v;
  bool isPumping = false;
  bool isPumpingIn = true;
};

}  // namespace gl::hw

#endif  // GL_HW_ACTUATORS_PUMP_H_