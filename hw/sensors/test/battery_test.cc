#include <gtest/gtest.h>

#include "hw/Pins.hh"
#include "hw/interfaces/mock/AdcMock.hh"
#include "hw/sensors/Battery.hh"

TEST(BatteryTest, Basic) {
  gl::hw::AdcMock adcMock;
  gl::hw::PinAnalogSensor bat1Pin = gl::hw::PinAnalogSensor::BATTERY1;
  gl::hw::PinAnalogSensor bat2Pin = gl::hw::PinAnalogSensor::BATTERY2;
  gl::hw::Battery battery1(&adcMock, bat1Pin, 4);
  gl::hw::Battery battery2(&adcMock, bat2Pin, 5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}