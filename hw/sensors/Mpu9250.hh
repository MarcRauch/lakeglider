#ifndef GL_HW_SENSORS_MPU9250_H
#define GL_HW_SENSORS_MPU9250_H

#include <stdint.h>

#include <array>
#include <optional>

#include "com/msg/Compass.hh"
#include "com/msg/Imu.hh"
#include "hw/interfaces/II2c.hh"
#include "utils/time/IClock.hh"

namespace gl::hw {
/**
 * Mpu9250 IMU and compass driver. As specified in
 * https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf
 */
class Mpu9250 {
 public:
  /**
   * Constructs a MPU9250 IMU / Compass object
   * @param[in] i2c I2c interface with the sensor connected
   * @param[in] clock A clock object used to wait
   * @param[in] addrImu the address of the IMU i2c interface
   * @param[in] addrCompass the address of the compass i2c interface
   * @returns MPU9250 object
   */
  Mpu9250(II2c& i2c, const utils::IClock& clock, uint8_t addrImu = 0x68, uint8_t addrCompass = 0x0C);

  /**
   * Initialize the MPU9250 sernsor
   * @returns true if the sensor was detected on the I2C bus. TODO: Add calibration here
   */
  bool initialize();

  /**
   * Capture an IMU measurement
   * @returns nullopt on failure, IMU measurement otherwise
   */
  std::optional<msg::Imu> getImuReading();

  /**
   * Capture a compass measurement
   * @returns nullopt on failure, compass measurement otherwise
   */
  std::optional<msg::Compass> getCompassReading();

 private:
  bool initializeImu();
  bool initializeCompass();
  bool isConnected();

  const uint8_t addrImu;
  const uint8_t addrCompass;
  II2c& i2c;
  const utils::IClock& clock;

  bool isInitialized = false;
  std::array<float, 3> compassAdjustments;
};
}  // namespace gl::hw

#endif  // GL_HW_SENSORS_MPU9250_H
