#include "hw/sensors/Mpu9250.hh"

#include <cstddef>

#include "com/msg/Compass.hh"
#include "utils/constants.hh"
#include "utils/time/Time.hh"

namespace {
// Registries IMU
constexpr uint8_t CMD_WHO_AM_I_IMU = 0x75;
constexpr uint8_t CMD_PWR_MGMT_1 = 0x6B;
constexpr uint8_t CMD_PWR_MGMT_2 = 0x6C;
constexpr uint8_t CMD_CONFIG = 0x1A;
constexpr uint8_t CMD_GYRO_CONFIG = 0x1B;
constexpr uint8_t CMD_ACC_CONFIG_1 = 0x1C;
constexpr uint8_t CMD_ACC_CONFIG_2 = 0x1D;
constexpr uint8_t CMD_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t CMD_INT_PIN_CF = 0x37;

// Registries Compass
constexpr uint8_t CMD_WHO_AM_I_COMPASS = 0x00;
constexpr uint8_t CMD_CTRL_1 = 0x0A;
constexpr uint8_t CMD_ASAX = 0x10;
constexpr uint8_t CMD_HXL = 0x03;
}  // namespace

namespace gl::hw {

Mpu9250::Mpu9250(II2c& i2c, const utils::IClock& clock, uint8_t addrImu, uint8_t addrCompass)
    : addrImu(addrImu), addrCompass(addrCompass), i2c(i2c), clock(clock), isInitialized(false) {}

bool Mpu9250::initialize() {
  // Reset
  bool success = i2c.writeCmd(addrImu, CMD_PWR_MGMT_1, 0b10000000);
  clock.wait(utils::Time::msec(100));
  // Activate imu
  success &= i2c.writeCmd(addrImu, CMD_PWR_MGMT_2, 0x00);
  // Activate compass
  success &= i2c.writeCmd(addrImu, CMD_INT_PIN_CF, 0b10);

  if (!success || !isConnected() || !initializeImu() || !initializeCompass()) {
    return false;
  };
  isInitialized = true;
  return true;
}

bool Mpu9250::initializeImu() {
  const auto modifyBits = [](uint8_t mask, uint8_t targetVal, uint8_t data) {
    return (data & ~mask) | (targetVal & mask);
  };

  // Set Bandwith of Gyro filter to 10Hz, this should give a smoother
  // measurement and should be enough for the slow motion of the glider
  uint8_t prevResult;
  bool success = i2c.readRegister(addrImu, CMD_CONFIG, std::span{reinterpret_cast<std::byte*>(&prevResult), 1});
  prevResult = modifyBits(0b111, 0b101, prevResult);
  success &= i2c.writeCmd(addrImu, CMD_CONFIG, prevResult);
  // Set the bandwith of the accelerometer filter to 20HZ
  success &= i2c.readRegister(addrImu, CMD_ACC_CONFIG_2, std::span{reinterpret_cast<std::byte*>(&prevResult), 1});
  prevResult = modifyBits(0b1111, 0b1100, prevResult);
  success &= i2c.writeCmd(addrImu, CMD_ACC_CONFIG_2, prevResult);
  // Set full scale of gyro to 500 degrees per second
  success &= i2c.readRegister(addrImu, CMD_GYRO_CONFIG, std::span{reinterpret_cast<std::byte*>(&prevResult), 1});
  prevResult = modifyBits(0b11000, 0b01000, prevResult);
  prevResult = modifyBits(0b11, 0b11, prevResult);
  success &= i2c.writeCmd(addrImu, CMD_GYRO_CONFIG, prevResult);
  // Set full scale of accelerometer to +/-4g
  success &= i2c.readRegister(addrImu, CMD_ACC_CONFIG_1, std::span{reinterpret_cast<std::byte*>(&prevResult), 1});
  prevResult = modifyBits(0b11000, 0b01000, prevResult);
  success &= i2c.writeCmd(addrImu, CMD_ACC_CONFIG_1, prevResult);

  success &= i2c.readRegister(addrImu, CMD_ACC_CONFIG_1, std::span{reinterpret_cast<std::byte*>(&prevResult), 1});
  return success;
}

bool Mpu9250::initializeCompass() {
  // Enter ROM access mode
  bool success = i2c.writeCmd(addrCompass, CMD_CTRL_1, 0x00);
  clock.wait(utils::Time::msec(100));
  success &= i2c.writeCmd(addrCompass, CMD_CTRL_1, 0x0F);
  clock.wait(utils::Time::msec(100));

  // Read adjustments
  std::array<std::byte, 3> calibrationBytes;
  success &= i2c.readRegister(addrCompass, CMD_ASAX, calibrationBytes);
  for (uint8_t i = 0; i < 3; i++) {
    compassAdjustments[i] = (static_cast<float>(calibrationBytes[i]) - 128.) / 256.f + 1.f;
  }

  // Reboot into continuous measurement 100Hz, 16Bit
  success &= i2c.writeCmd(addrCompass, CMD_CTRL_1, 0x00);
  clock.wait(utils::Time::msec(100));
  success &= i2c.writeCmd(addrCompass, CMD_CTRL_1, 0b10110);
  clock.wait(utils::Time::msec(100));
  return success;
}

bool Mpu9250::isConnected() {
  uint8_t responseImu;
  if (!i2c.readRegister(addrImu, CMD_WHO_AM_I_IMU, std::span{reinterpret_cast<std::byte*>(&responseImu), 1})) {
    return false;
  }
  uint8_t responseCompass;
  if (!i2c.readRegister(addrCompass, CMD_WHO_AM_I_COMPASS,
                        std::span{reinterpret_cast<std::byte*>(&responseCompass), 1})) {
    return false;
  }
  return responseImu == 0x71 && responseCompass == 0x48;
}

std::optional<msg::Imu> Mpu9250::getImuReading() {
  using utils::consts::DEG_TO_RAD;
  using utils::consts::G_M_S2;

  msg::Imu result;
  std::array<std::byte, 14> imuBytes;
  if (!i2c.readRegister(addrImu, CMD_ACCEL_XOUT_H, imuBytes)) {
    return std::nullopt;
  }

  const auto combineBytes = [](std::byte msb, std::byte lsb) {
    const uint16_t high = static_cast<uint16_t>(msb);
    const uint16_t low = static_cast<uint16_t>(lsb);
    const uint16_t combined = (high << 8) | low;
    return static_cast<int16_t>(combined);
  };
  // This assumes a max acc of +/-4g and max w of 100deg/sec
  result.acc_m_s2[0] = G_M_S2 * combineBytes(imuBytes[0], imuBytes[1]) * 4. / 32768.;
  result.acc_m_s2[1] = G_M_S2 * combineBytes(imuBytes[2], imuBytes[3]) * 4. / 32768.;
  result.acc_m_s2[2] = G_M_S2 * combineBytes(imuBytes[4], imuBytes[5]) * 4. / 32768.;

  result.w_rad_s[0] = DEG_TO_RAD * combineBytes(imuBytes[8], imuBytes[9]) * 500. / 32768.;
  result.w_rad_s[1] = DEG_TO_RAD * combineBytes(imuBytes[10], imuBytes[11]) * 500. / 32768.;
  result.w_rad_s[2] = DEG_TO_RAD * combineBytes(imuBytes[12], imuBytes[13]) * 500. / 32768.;

  return result;
}

std::optional<msg::Compass> Mpu9250::getCompassReading() {
  msg::Compass result;
  std::array<std::byte, 7> measBytes;
  bool success = i2c.readRegister(addrCompass, CMD_HXL, measBytes);
  success &= ~(static_cast<uint8_t>(measBytes[6]) & 0b1000);
  if (!success) {
    return std::nullopt;
  }

  const int16_t mx = static_cast<uint16_t>(measBytes[0]) | (static_cast<uint16_t>(measBytes[1]) << 8);
  const int16_t my = static_cast<uint16_t>(measBytes[2]) | (static_cast<uint16_t>(measBytes[3]) << 8);
  const int16_t mz = static_cast<uint16_t>(measBytes[4]) | (static_cast<uint16_t>(measBytes[5]) << 8);

  // This depends on the resolution to be set to 16 bits
  result.mag_ut[0] = (static_cast<double>(mx) * 4912. * compassAdjustments[0] / 32760.);
  result.mag_ut[1] = (static_cast<double>(my) * 4912. * compassAdjustments[1] / 32760.);
  result.mag_ut[2] = (static_cast<double>(mz) * 4912. * compassAdjustments[2] / 32760.);

  return result;
}
}  // namespace gl::hw
