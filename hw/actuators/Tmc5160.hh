#ifndef GL_HW_ACTUATORS_TMC5160_H_
#define GL_HW_ACTUATORS_TMC5160_H_

#include <stdint.h>
#include <concepts>

#include "hw/Pins.hh"
#include "hw/interfaces/ISpi.hh"
#include "utils/time/IClock.hh"

// Implemented according to https://www.analog.com/media/en/technical-documentation/data-sheets/tmc5160a_datasheet_rev1.17.pdf

namespace gl::hw {

namespace stepperDetails {
// Requires a valid register (contains a uint32_t bytes field)
template <typename T>
concept ConceptRegister = requires(T t) {
  { t.bytes } -> std::same_as<uint32_t&>;
};
}  // namespace stepperDetails

class Stepper {
 public:
  struct Config {
    double iRun_a = 0.2;
    double iHoldFactor = 0.3;
    bool invertDirection = false;
    double vel_radps = 6.;
    double acc_radps2 = 2.;
    double maxRotations = 2.;
  };

  /**
   * Create a Stepper object with config 
   * @param[in] config Stepper config to be used
   * @returns Stepper object
   */
  Stepper(ISpi* spi, utils::IClock* clock, const Config& config);

  /**
   * Move in the negative direction until a limit switch is hit. Set the zero position and move to the middle of the 
   * range
   */
  void home();

  /**
   * Move to a position. The position is to be specified in [0,1]. where 0 is the start and 1 the end of the range
   * @param[in] pos Position to move to [0,1]
   */
  void moveTo(double pos);

 private:
  enum class Register : uint8_t {
    GCONF = 0x00,
    GSTAT = 0x01,
    GLOBAL_SCALER = 0x0B,
    IHOLD_IRUN = 0x10,
    RAMPMODE = 0x20,
    XACTUAL = 0x21,
    VACTUAL = 0x22,
    VSTART = 0x23,
    V1 = 0x25,
    AMAX = 0x26,
    VMAX = 0x27,
    DMAX = 0x28,
    D1 = 0x2A,
    VSTOP = 0x2B,
    XTARGET = 0x2D,
    SW_MODE = 0x34,
    RAMP_STAT = 0x35,
    CHOP_CONF = 0x6C,
    PWM_CONF = 0x70,
  };

  /**
   * Update register value at a given address
   * @param[in] addr Address of register to be updated
   * @param[in] val Value to be written to the register
   */
  template <stepperDetails::ConceptRegister T>
  void writeRegister(Register addr, T val) {
    const uint32_t bytes = val.bytes;
    uint8_t bufSend[5];
    bufSend[0] = static_cast<uint32_t>(addr) | 1 << 7;
    bufSend[4] = static_cast<uint8_t>(bytes & 0xFF);
    bufSend[3] = static_cast<uint8_t>((bytes >> 8) & 0xFF);
    bufSend[2] = static_cast<uint8_t>((bytes >> 16) & 0xFF);
    bufSend[1] = static_cast<uint8_t>((bytes >> 24) & 0xFF);
    spi->writeBytes(bufSend, 5);
  }

  /**
   * Read register value at a given address. 
   * @param[in] addr Address of register to be read
   * @returns Value in this register.
   */
  template <stepperDetails::ConceptRegister T>
  T readRegister(Register addr) {
    uint8_t bufSend[5];
    uint8_t bufRecv[5];
    bufSend[0] = static_cast<uint8_t>(addr);
    // Read twice, the first time requests the value, the second time actually reads it
    spi->writeBytes(bufSend, 5);
    spi->readBytes(5, bufRecv);
    spi->writeBytes(bufSend, 5);
    spi->readBytes(5, bufRecv);
    uint32_t dat = static_cast<uint32_t>(bufRecv[1]);
    dat = (dat << 8) | static_cast<uint32_t>(bufRecv[2]);
    dat = (dat << 8) | static_cast<uint32_t>(bufRecv[3]);
    dat = (dat << 8) | static_cast<uint32_t>(bufRecv[4]);
    return T{.bytes = dat};
  }

  const Config config;
  ISpi* spi;
  utils::IClock* clock;
};

}  // namespace gl::hw

#endif  // GL_HW_ACTUATORS_TMC5160_H_