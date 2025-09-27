#include "hw/actuators/EqiUartMg1.hh"

namespace gl::hw {

EqiUartMg1::EqiUartMg1(IUart* uart) : uart(uart) {
  // Set full speed as default.
  setSpeed(1.);
  setDirection(true);
  stop();
}

void EqiUartMg1::setDirection(bool pumpIn) {
  const std::string dirCmd = pumpIn ? "ZDInvSet = 3\r" : "ZDInvSet = 1\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(dirCmd.c_str()), dirCmd.size());
}

void EqiUartMg1::setSpeed(double speed) {
  const uint32_t speedUint = std::min<uint32_t>(std::max<uint32_t>(speed * 1000, 0), 1000);
  const std::string speedCmd = "ZDDDSMDC = " + std::to_string(speedUint) + "\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(speedCmd.c_str()), speedCmd.size());
}

void EqiUartMg1::start() {
  const std::string startCmd = "ZDddstart\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(startCmd.c_str()), startCmd.size());
}

void EqiUartMg1::stop() {
  const std::string stopCmd = "ZDddstop\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(stopCmd.c_str()), stopCmd.size());
}
}  // namespace gl::hw