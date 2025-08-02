#ifndef GL_HW_INTERFACES_PICO_I2CPICO_H_
#define GL_HW_INTERFACES_PICO_I2CPICO_H_

#include <hardware/i2c.h>
#include <stdint.h>

#include "hw/Pins.hh"
#include "hw/interfaces/II2c.hh"

namespace gl::hw {

/**
Implements the I2c interface for the rp2350
*/
class I2cPico : public II2c {
 public:
  /**
   * Constructs a I2c object and initializes it
   * @param[in] i2cInst I2c instance to be used
   * @param[in] pinScl hardware pin number for the i2c clock
   * @param[in] pinSda hardware pin number for the i2c data
   * @returns I2c object
   */
  I2cPico(i2c_inst_t* ic2Inst, PinGpioSensor pinScl, PinGpioSensor pinSda);

  bool readBytes(uint8_t address, uint8_t numBytes, uint8_t* dest) override;

  bool readRegister(uint8_t address, uint8_t reg, uint8_t numBytes, uint8_t* dest) override;

  bool writeBytes(uint8_t address, const uint8_t* data, uint8_t numBytes) override;

  bool writeCmd(uint8_t address, uint8_t cmd, const uint8_t* data, uint8_t numBytes) override;

  bool writeCmd(uint8_t address, uint8_t cmd, uint8_t data) override;

 private:
  i2c_inst_t* i2cInst = nullptr;
  const PinGpioSensor pinScl;
  const PinGpioSensor pinSda;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_I2CPICO_H_
