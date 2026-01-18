#include "hw/sensors/Ms5837.hh"

#include <stdlib.h>

#include <vector>

#include "com/msg/Depth.hh"

namespace {
// Registries on the MS5837
constexpr uint8_t CMD_RESET = 0x1E;
constexpr uint8_t CMD_PROM = 0xA0;
constexpr uint8_t CMD_CONVERT_D1 = 0x48;
constexpr uint8_t CMD_CONVERT_D2 = 0x58;
constexpr uint8_t CMD_ADC = 0x00;

// Calculates the CRC to verify pressure sensor measurements
uint8_t crc4(uint16_t* data) {
  uint16_t rem = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (i % 2 == 1)
      rem ^= data[i >> 1] & 0x00FF;
    else
      rem ^= data[i >> 1] >> 8;
    for (uint8_t j = 8; j > 0; j--) {
      if (rem & 0x8000)
        rem = (rem << 1) ^ 0x3000;
      else
        rem = rem << 1;
    }
  }
  rem = (rem >> 12) & 0x000F;
  return rem ^ 0x00;
}

// Read the content of the Prom used for calibration
std::optional<uint16_t> readProm(uint8_t i2cAddr, gl::hw::II2c& i2c, uint8_t promAddr) {
  std::array<std::byte, 2> buf;
  if (!i2c.readRegister(i2cAddr, promAddr, buf)) {
    return std::nullopt;
  }
  // MSB is sent first
  return (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]);
}
}  // namespace

namespace gl {
namespace hw {

Ms5837::Ms5837(II2c& i2c, const utils::IClock& clock, uint8_t i2cAddr) : i2cAddr(i2cAddr), i2c(i2c), clock(clock) {
  for (uint8_t i = 0; i < 10; i++) {
    if (initialize()) {
      isInitialized = true;
      break;
    }
  }
}

bool Ms5837::initialize() {
  if (!i2c.writeBytes(i2cAddr, std::array{static_cast<std::byte>(CMD_RESET)})) {
    return false;
  }
  // Wait for reset
  clock.wait(utils::Time::msec(100));
  for (uint8_t i = 0; i < 7; i++) {
    const std::optional<uint16_t> promVal = readProm(i2cAddr, i2c, CMD_PROM + 2 * i);
    if (!promVal.has_value()) {
      return false;
    }
    c[i] = *promVal;
  }
  uint8_t crcExpected = c[0] >> 12;
  c[0] &= 0x0FFF;
  c[7] = 0;
  return crcExpected == crc4(c);
}

std::optional<msg::Depth> Ms5837::loop() {
  utils::Time now = clock.now();
  // Start first convertion
  if (!convertionTime1.has_value() && !convertionTime2.has_value()) {
    if (i2c.writeBytes(i2cAddr, std::array{static_cast<std::byte>(CMD_CONVERT_D1)})) {
      convertionTime1 = now;
    }
    return std::nullopt;
  }
  // If the first convertion is done read value and start the second one
  if (convertionTime1.has_value()) {
    if ((now - *convertionTime1).msec<double>() > 10) {
      convertionTime1.reset();
      std::array<std::byte, 3> buf;
      if (!i2c.readRegister(i2cAddr, CMD_ADC, buf)) {
        return std::nullopt;
      }
      d1 = (static_cast<uint32_t>(buf[0]) << 16) | (static_cast<uint32_t>(buf[1]) << 8) | static_cast<uint32_t>(buf[2]);
      if (i2c.writeBytes(i2cAddr, std::array{static_cast<std::byte>(CMD_CONVERT_D2)})) {
        convertionTime2 = now;
      }
    }
    return std::nullopt;
  }
  // If the second convertion is done, read value and return message
  if (convertionTime2.has_value()) {
    if ((now - *convertionTime2).msec<double>() > 10) {
      convertionTime2.reset();
      std::array<std::byte, 3> buf;
      if (!i2c.readRegister(i2cAddr, CMD_ADC, buf)) {
        return std::nullopt;
      }
      const double d2 =
          (static_cast<uint32_t>(buf[0]) << 16) | (static_cast<uint32_t>(buf[1]) << 8) | static_cast<uint32_t>(buf[2]);

      // The result is only 0 if the read failed
      if (d1 == 0 || d2 == 0) {
        return std::nullopt;
      }
      int32_t dT = static_cast<int32_t>(d2) - (static_cast<int32_t>(c[5]) << 8);
      int32_t temp = 2000 + ((static_cast<int64_t>(dT) * static_cast<int64_t>(c[6])) >> 23);

      int64_t off = (static_cast<int64_t>(c[2]) << 16) + ((static_cast<int64_t>(c[4]) * dT) >> 7);
      int64_t sens = (static_cast<int64_t>(c[1]) << 15) + ((static_cast<int64_t>(c[3]) * dT) >> 8);

      int64_t offi;
      int64_t sensi;
      // second order
      if (temp / 100 < 20) {
        dT = (3 * static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) >> 33;
        offi = (3 * static_cast<int64_t>(temp - 2000) * static_cast<int64_t>(temp - 2000)) >> 1;
        sensi = (5 * static_cast<int64_t>(temp - 2000) * static_cast<int64_t>(temp - 2000)) >> 3;
        if ((temp / 100) < -15) {
          offi += 7 * (static_cast<int64_t>(temp) + 1500) * (static_cast<int64_t>(temp) + 1500);
          sensi += 4 * (static_cast<int64_t>(temp) + 1500) * (static_cast<int64_t>(temp) + 1500);
        }
      } else {
        dT = (static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) >> 36;
        offi = (static_cast<int64_t>(temp - 2000) * static_cast<int64_t>(temp - 2000)) >> 4;
        sensi = 0;
      }
      const int64_t off2 = off - offi;
      const int64_t sens2 = sens - sensi;
      msg::Depth msg;
      msg.temperature_degc = (temp - dT) / 100.;
      msg.pressure_pa = ((((static_cast<int64_t>(d1) * sens2) >> 21) - off2) >> 13) * 10.;
      msg.depth_m = msg.pressure_pa / (9.81 * 997.);
      return msg;
    }
  }
  return std::nullopt;
}

}  // namespace hw
}  // namespace gl
