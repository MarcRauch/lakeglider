#include "hw/actuators/Pump.hh"

namespace {
// Percentage of full range (plus and minus) to accept the fullness
constexpr double THRESHOLD_FAC = 0.1;
}  // namespace

namespace gl::hw {
void Pump::pumpTo(double bottleLevel) {
  targetLevel_v = refEmpty_v + bottleLevel * (refFull_v - refEmpty_v);
}

void Pump::loop() {
  const double threshold_v = THRESHOLD_FAC * (refFull_v - refEmpty_v);
  if (targetLevel_v - readPotentiometer() > threshold_v) {
    if (isPumpingIn) {
      if (isPumping) {
        stopPump();
        return;
      }
      setPumpDirection(false);
      return;
    }
    if (!isPumping) {
      startPump();
      return;
    }
  }
  if (targetLevel_v - readPotentiometer() < -threshold_v) {
    if (!isPumpingIn) {
      if (isPumping) {
        stopPump();
        return;
      }
      setPumpDirection(true);
      return;
    }
    if (!isPumping) {
      startPump();
      return;
    }
  }
  stopPump();
}

void Pump::setPumpDirection(bool pumpIn) {
  const std::string dirCmd = pumpIn ? "ZDInvSet = 3\r" : "ZDInvSet = 1\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(dirCmd.c_str()), dirCmd.size());
}

void Pump::openValve() {
  gpioValve->setHigh();
}

void Pump::closeValve() {
  gpioValve->setLow();
}

void Pump::startPump() {
  const std::string startCmd = "ZDddstart\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(startCmd.c_str()), startCmd.size());
  openValve();
  isPumping = true;
}

void Pump::stopPump() {
  const std::string stopCmd = "ZDddstop\r";
  uart->writeBytes(reinterpret_cast<const uint8_t*>(stopCmd.c_str()), stopCmd.size());
  closeValve();
  isPumping = false;
}

double Pump::readPotentiometer() {
  return adcPotentiometer->read();
}
}  // namespace gl::hw