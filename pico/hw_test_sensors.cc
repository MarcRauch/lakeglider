#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "pico/stdlib.h"

#include "com/msg/Battery.hh"
#include "com/msg/Depth.hh"
#include "hw/interfaces/pico/AdcPico.hh"
#include "hw/interfaces/pico/I2cPico.hh"
#include "hw/sensors/Battery.hh"
#include "hw/sensors/Ms5837.hh"
#include "utils/time/PicoClock.hh"

void testBattery(gl::hw::Battery& b) {
  printf("Starting battery test\n");
  const std::optional<gl::msg::Battery> reading = b.getReading();
  if (!reading.has_value()) {
    std::cout << "Battery reading failed" << std::endl;
  }
  printf("Total voltage: %fV\n", reading->voltage_v);
  printf("Percentage: %f%\n", reading->percentage);
  printf("Cell Voltage: %fV\n", reading->cellVoltage_v);
  printf("Done\n");
}

void testDepth(gl::hw::Ms5837& depth) {
  printf("Starting depth test\n");
  std::optional<gl::msg::Depth> msg = depth.loop();
  while (!msg.has_value()) {
    sleep_ms(10);
  }
  printf("Depth: %fm\n", msg->depth_m);
  printf("Temp: %f°C\n", msg->temperature_degc);
  printf("Pressure: %fpa\n", msg->pressure_pa);
  printf("Done\n");
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
  printf("Starting...\n");

  // init HW
  gl::hw::AdcPico adcBat1(gl::hw::PinAnalogSensor::BATTERY1);
  gl::hw::AdcPico adcBat2(gl::hw::PinAnalogSensor::BATTERY2);
  i2c_inst_t* i2cInst = i2c_get_instance(gl::hw::SENSOR_I2C_INSTANCE_NR);
  gl::hw::I2cPico i2cPico(i2cInst, gl::hw::PinGpioSensor::I2C_SCL, gl::hw::PinGpioSensor::I2C_SDA);
  gl::utils::PicoClock clock;

  gl::hw::Battery bat1(adcBat1, clock);
  gl::hw::Battery bat2(adcBat2, clock);
  gl::hw::Ms5837 depth(i2cPico, clock);

  uint8_t i = 0;
  for (; !depth.initialize() && i < 10; i++) {
    printf("Depth sensor init failed attempt %u/10\n", i + 1);
    sleep_ms(200);
  }
  if (i == 10) {
    printf("Depth sensor init failed, will not be available.\n");
  }

  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_RED), false);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_GREEEN), true);
  printf("Hardware test, to start component type battery or depth\n");

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
        printf("Testing Battery 1\n");
        testBattery(bat1);
        printf("Testing Battery 2\n");
        testBattery(bat2);
      } else if (strcmp(buffer, "depth") == 0) {
        testDepth(depth);
      } else {
        printf("Unknown command %s\n", buffer);
      }
    }
  }
}
