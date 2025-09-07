#ifndef GL_HW_INTERFACES_PICO_SPIPICO_H_
#define GL_HW_INTERFACES_PICO_SPIPICO_H_

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <stdint.h>

#include <type_traits>

#include "hw/Pins.hh"
#include "hw/interfaces/ISpi.hh"

namespace gl::hw {

/**
Implements the SPI interface for the rp2350
*/
class SpiPico : public ISpi {
 public:
  /**
   * Constructs a I2c object and initializes it
   * @param[in] spiInst I2c instance to be used
   * @param[in] pinMiso hardware pin number for the MISO connection
   * @param[in] pinMosi hardware pin number for the MOSI connection
   * @param[in] pinSck hardware pin number for the Sck connection
   * @param[in] pinCs hardware pin number for the chip select connection
   * @returns SPI object
   */
  template <ConceptPinGpio T>
  SpiPico(spi_inst_t* spiInst, T pinMiso, T pinMosi, T pinSck, T pinCs) : pinCs(static_cast<uint8_t>(pinCs)) {
    if (!isInitialized) {
      spi_init(spiInst, 1000000);
      gpio_set_function(static_cast<uint8_t>(pinMiso), GPIO_FUNC_SPI);
      gpio_set_function(static_cast<uint8_t>(pinMiso), GPIO_FUNC_SPI);
      gpio_set_function(static_cast<uint8_t>(pinSck), GPIO_FUNC_SPI);
    }
    gpio_init(this->pinCs);
    gpio_set_dir(this->pinCs, GPIO_OUT);
    gpio_put(this->pinCs, 1);
    setInitialized();
  }

  bool readBytes(uint8_t numBytes, uint8_t* dest) override;
  bool writeBytes(const uint8_t* data, uint8_t numBytes) override;

 private:
  static void setInitialized();
  void select();
  void deselect();

  spi_inst_t* spiInst = nullptr;
  const uint8_t pinCs;

  inline static bool isInitialized = false;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_SPIPICO_H_
