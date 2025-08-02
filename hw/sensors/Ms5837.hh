#ifndef GL_HW_SENSORS_MS5837_H
#define GL_HW_SENSORS_MS5837_H

#include <stdint.h>

#include "com/msg/Depth.hh"
#include "hw/interfaces/II2c.hh"
#include "utils/time/ITimeProvider.hh"

namespace gl {
namespace hw {
/**
MS5837 Pressure sensor driver. As specified in
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5837-30BA%7FB1%7Fpdf%7FEnglish%7FENG_DS_MS5837-30BA_B1.pdf%7FCAT-BLPS0017
*/
class Ms5837 {
 public:
  /**
   * Constructs a MS5837 pressure sensor object
   * @param[in] i2c Pointer to i2c interface used for communication with the sensor
   * @param[in] timeProvider Pointer to generic time provider to be used
   * @param[in] i2cAddr I2C address of the sensor
   * @returns MS5837 pressure sensor object
   */
  Ms5837(II2c* i2c, const utils::ITimeProvider* timeProvider, uint8_t i2cAddr = 0x76);

  /**
   * Initialize the MS5837 sensor.
   * @returns true if the sensor was detected on the I2C bus. TODO: Add calibration here
   */
  bool initialize();

  /**
   * Get a pressure measurement (temperature, pressure and depth)
   * @param[out] msg The depth message to fill
   * @returns True if the read was successful
   */
  bool getReading(msg::Depth* msg) const;

 private:
  const uint8_t i2cAddr;
  II2c* i2c;
  const utils::ITimeProvider* timeProvider;

  bool isInitialized = false;
  uint16_t c[8];
};
}  // namespace hw
}  // namespace gl

#endif  // GL_HW_SENSORS_MS5837_H
