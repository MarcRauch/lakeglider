#ifndef GL_HW_INTERFACES_PICO_I2CPICO_H_
#define GL_HW_INTERFACES_PICO_I2CPICO_H_

#include <hardware/gpio.h>
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
  template <ConceptPinGpio T>
  I2cPico(i2c_inst_t* i2cInst, T pinScl, T pinSda) : i2cInst(i2cInst) {
    // TODO: Do this only once per interface, maybe let the user set the speed
    gpio_set_function(static_cast<uint8_t>(pinScl), GPIO_FUNC_I2C);
    gpio_set_function(static_cast<uint8_t>(pinSda), GPIO_FUNC_I2C);
    gpio_pull_up(static_cast<uint8_t>(pinScl));
    gpio_pull_up(static_cast<uint8_t>(pinSda));
    i2c_init(i2cInst, 100 * 1000);
  }

  bool readBytes(uint8_t address, std::span<std::byte> data) override;

  bool readRegister(uint8_t address, uint8_t reg, std::span<std::byte> data) override;

  bool writeBytes(uint8_t address, std::span<const std::byte> data) override;

  bool writeCmd(uint8_t address, uint8_t cmd, std::span<const std::byte> data) override;

  bool writeCmd(uint8_t address, uint8_t cmd, uint8_t data) override;

 private:
  i2c_inst_t* i2cInst = nullptr;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_I2CPICO_H_
