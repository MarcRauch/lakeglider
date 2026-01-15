#ifndef GL_HW_SENSORS_BATTERY_H_
#define GL_HW_SENSORS_BATTERY_H_

#include <stdint.h>

#include <optional>

#include "com/msg/Battery.hh"
#include "hw/interfaces/IAdc.hh"
#include "utils/time/IClock.hh"

namespace gl::hw {

/**
Battery monitor. Does not look at individual cells.
*/
class Battery {
 public:
  /**
   * Constructs a battery monitor object
   * @param[in] iAdc ADC interface used to read the battery voltage.
   * @param[in] numCells Number of cells of the LiPo battery.
   * @returns Corresponding battery monitor object.
   */
  Battery(const IAdc& iAdc, const utils::IClock& clock, uint8_t numCells = 4);

  /**
   * Get battery charge information. This includes, total voltage, cell voltage and charge percentage.
   * @returns Measurement in case of successful reading, nullopt otherwise.
   */
  std::optional<msg::Battery> getReading() const;

 private:
  const IAdc& iAdc;
  const utils::IClock& clock;
  const uint8_t numCells;

  float maxVoltage_v;
  float minVoltage_v;
};

}  // namespace gl::hw

#endif  // GL_HW_SENSORS_BATTERY_H_
