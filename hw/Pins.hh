#ifndef HW_SENSORS_PINS_H_
#define HW_SENSORS_PINS_H_

#include <stdint.h>

namespace gl::hw {

/**
* Pin numbers of connected digital electrical components of the sensor board.
*/
enum class PinGpioSensor : uint8_t {
  LED_RED = 6,
  LED_GREEEN = 7,
  SPI_MISO = 16,
  SPI_CS = 17,
  SPI_SCK = 18,
  SPI_MOSI = 19,
  I2C_SDA = 28,
  I2C_SCL = 29,
};

/**
* Pin numbers of connected analog electrical components of the sensor board.
*/
enum class PinAnalogSensor : uint8_t {
  BATTERY1 = 0,
  BATTERY2 = 1,
};

// Interface numbers
const uint8_t SENSOR_I2C_INSTANCE_NR = 0;
const uint8_t SENSOR_SPI_INSTANCE_NR = 0;

}  // namespace gl::hw

#endif  // HW_SENSORS_PINS_H_