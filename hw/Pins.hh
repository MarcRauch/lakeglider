#ifndef HW_PINS_H_
#define HW_PINS_H_

#include <stdint.h>

#include <type_traits>

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
enum class PinAnalogSensor : uint8_t { BATTERY1 = 0, BATTERY2 = 1 };

/**
* Pin numbers of connected digital electrical components of the actuator board.
*/
enum class PinGpioActuator : uint8_t {
  LED_RED = 6,
  LED_GREEEN = 7,
  PUMP_TX = 12,
  PUMP_RX = 13,
  SPI_MISO = 16,
  SPI_CS_CAN = 17,
  SPI_SCK = 18,
  SPI_MOSI = 19,
  SPI_CS_MOT2 = 20,
  SPI_CS_MOT1 = 21,
  VALVE = 28,
};

/**
* Pin nubers of connected digital electrical components to the compute board.
*/
enum class PinGpioCompute : uint8_t {
  LED_RED = 5,
  LED_GREEN = 6,
  SPI_CS = 8,
  SPI_MISO = 9,
  SPI_MOSI = 10,
  SPI_SCK = 11
};

/**
* Pin numbers of connected analog electrical components of the sensor board.
*/
enum class PinAnalogActuator : uint8_t { PUMP_MEAS = 3 };

enum class CanId : uint8_t { NONE = 0, COMPUTE_BOARD = 1, SENSOR_BOARD = 2, ACTUATOR_BOARD = 3 };

// Interface numbers
const uint8_t SENSOR_I2C_INSTANCE_NR = 0;
const uint8_t SENSOR_SPI_INSTANCE_NR = 0;
const uint8_t ACTUATOR_SPI_INSTANCE_NR = 0;
const uint8_t ACTUATOR_UART_INSTANCE_NR = 0;

template <typename T>
concept ConceptPinGpio =
    std::is_same_v<T, PinGpioSensor> || std::is_same_v<T, PinGpioActuator> || std::is_same_v<T, PinGpioCompute>;
template <typename T>
concept ConceptPinAnalog =
    std::is_same_v<T, PinAnalogSensor> || std::is_same_v<T, PinAnalogActuator> || std::is_same_v<T, PinGpioCompute>;
}  // namespace gl::hw

#endif  // HW_PINS_H_
