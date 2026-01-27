#include <pico/stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

#include "com/msg/Battery.hh"
#include "com/msg/Compass.hh"
#include "com/msg/Depth.hh"
#include "com/msg/Imu.hh"
#include "hw/interfaces/pico/AdcPico.hh"
#include "hw/interfaces/pico/I2cPico.hh"
#include "hw/sensors/Battery.hh"
#include "hw/sensors/Mpu9250.hh"
#include "hw/sensors/Ms5837.hh"
#include "utils/constants.hh"
#include "utils/time/PicoClock.hh"

void testBattery(gl::hw::Battery& b) {
  std::cout << "Starting battery test" << std::endl;
  const std::optional<gl::msg::Battery> reading = b.getReading();
  if (!reading.has_value()) {
    std::cout << "Battery reading failed" << std::endl;
  }
  std::cout << "Total voltage: " << reading->voltage_v << "V" << std::endl;
  std::cout << "Percentage: " << reading->percentage << std::endl;
  std::cout << "Cell Voltage: " << reading->cellVoltage_v << "V" << std::endl;
  std::cout << "Done" << std::endl;
}

void testDepth(gl::hw::Ms5837& depth) {
  std::cout << "Starting depth test" << std::endl;
  std::optional<gl::msg::Depth> msg = depth.loop();
  while (!msg.has_value()) {
    sleep_ms(10);
    msg = depth.loop();
  }
  std::cout << "Depth: " << msg->depth_m << "m" << std::endl;
  std::cout << "Temp: " << msg->temperature_degc << "°C" << std::endl;
  std::cout << "Pressure: " << msg->pressure_pa << "pa" << std::endl;
  std::cout << "Done" << std::endl;
}

void testImu(gl::hw::Mpu9250& mpu9250) {
  using gl::utils::consts::RAD_TO_DEG;
  std::cout << "starting IMU test" << std::endl;
  std::optional<gl::msg::Imu> imuMsg = mpu9250.getImuReading();
  std::optional<gl::msg::Compass> compassMsg = mpu9250.getCompassReading();
  if (imuMsg) {
    std::cout << "acceleration [m/s2] x:" << imuMsg->acc_m_s2[0] << " y: " << imuMsg->acc_m_s2[1]
              << " z: " << imuMsg->acc_m_s2[2] << std::endl;
    std::cout << "rotation rate [°/s] x:" << imuMsg->w_rad_s[0] * RAD_TO_DEG
              << " y: " << imuMsg->w_rad_s[1] * RAD_TO_DEG << " z: " << imuMsg->w_rad_s[2] * RAD_TO_DEG << std::endl;
  } else {
    std::cout << "Failed to read IMU" << std::endl;
  }
  if (compassMsg) {
    std::cout << "Mag. Field [uT] x:" << compassMsg->mag_ut[0] << " y: " << compassMsg->mag_ut[1]
              << " z: " << compassMsg->mag_ut[2] << std::endl;
  } else {
    std::cout << "Failed to read compass" << std::endl;
  }
}

void prepareCmdString(char* buffer) {
  for (uint32_t i = 0; buffer[i] != '\0'; i++) {
    if (buffer[i] == ' ' || buffer[i] == '\n') {
      buffer[i] = '\0';
    }
  }
}

void advanceCmdString(char* buffer, uint32_t lenBuffer) {
  uint8_t offset = 0;
  for (; buffer[offset] != '\0' && offset < lenBuffer; offset++) {}
  offset++;
}

int main() {
  stdio_init_all();
  gpio_init(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_RED));
  gpio_init(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_GREEEN));
  gpio_set_dir(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_RED), GPIO_OUT);
  gpio_set_dir(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_GREEEN), GPIO_OUT);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_RED), true);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_GREEEN), false);
  sleep_ms(4000);
  std::cout << "Starting..." << std::endl;

  // init HW
  gl::hw::AdcPico adcBat1(gl::hw::PinAnalogSensor::BATTERY1);
  gl::hw::AdcPico adcBat2(gl::hw::PinAnalogSensor::BATTERY2);
  i2c_inst_t* i2cInst = i2c_get_instance(gl::hw::SENSOR_I2C_INSTANCE_NR);
  gl::hw::I2cPico i2cPico(i2cInst, gl::hw::PinGpioSensor::I2C_SCL, gl::hw::PinGpioSensor::I2C_SDA);
  gl::utils::PicoClock clock;

  gl::hw::Battery bat1(adcBat1, clock);
  gl::hw::Battery bat2(adcBat2, clock);
  gl::hw::Ms5837 depth(i2cPico, clock);
  gl::hw::Mpu9250 mpu9250(i2cPico, clock);

  uint8_t i = 0;
  for (; !depth.initialize() && i < 10; i++) {
    std::cout << "Depth sensor init failed attempt " << i + 1 << "/10" << std::endl;
    sleep_ms(200);
  }
  if (i == 10) {
    std::cout << "Depth sensor init failed, will not be available." << std::endl;
  }
  if (!mpu9250.initialize()) {
    std::cout << "IMU init failed, will not be available" << std::endl;
  }

  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_RED), false);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_GREEEN), true);
  std::cout << "Hardware test, to start component type battery, imu or depth" << std::endl;

  char buffer[128];
  while (true) {
    memset(buffer, '\0', sizeof(buffer));
    if (fgets(buffer, sizeof(buffer), stdin) != NULL) {
      if (buffer[0] == '\0') {
        memmove(buffer, buffer + 1, sizeof(buffer) - 1);
      }
      prepareCmdString(buffer);

      // Match to commands
      if (strcmp(buffer, "battery") == 0) {
        std::cout << "Testing Battery 1" << std::endl;
        testBattery(bat1);
        std::cout << "Testing Battery 2" << std::endl;
        testBattery(bat2);
      } else if (strcmp(buffer, "depth") == 0) {
        testDepth(depth);
      } else if (strcmp(buffer, "imu") == 0) {
        testImu(mpu9250);
      } else {
        printf("Unknown command %s\n", buffer);
      }
    }
  }
}
