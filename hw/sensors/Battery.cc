#include "hw/sensors/Battery.hh"

#include "hw/Pins.hh"

namespace {
constexpr double MAX_CELL_VOLTAGE_V = 4.2;
constexpr double MIN_CELL_VOLTAGE_V = 3.2;
constexpr double REF_VOLTAGE_V = 3.3;
constexpr double R1_OHM = 24000;
constexpr double R2_OHM = 5000;
}  // namespace

namespace gl::hw {

Battery::Battery(const IAdc* iAdc, const utils::IClock* clock, PinAnalogSensor pin, uint8_t numCells)
    : iAdc(iAdc), clock(clock), pin(pin), numCells(numCells) {
  maxVoltage_v = numCells * MAX_CELL_VOLTAGE_V;
  minVoltage_v = numCells * MIN_CELL_VOLTAGE_V;
}

bool Battery::getReading(msg::Battery* msg) const {
  double adcResult = iAdc->read(pin);

  const float voltage_v = adcResult * REF_VOLTAGE_V * (R1_OHM + R2_OHM) / R2_OHM;
  msg->voltage_v = voltage_v;
  msg->cellVoltage_v = voltage_v / numCells;
  msg->percentage = 100.0f * (voltage_v - minVoltage_v) / (maxVoltage_v - minVoltage_v);
  msg->timestamp_us = tclock->now().usec<uint64_t>();
  return true;
}

}  // namespace gl::hw