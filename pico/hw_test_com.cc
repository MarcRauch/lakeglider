#include <pico/stdlib.h>

#include <cstdint>
#include <iostream>

#include "com/msg/Battery.hh"
#include "com/msg/Header.hh"
#include "com/msg/utils.hh"
#include "hw/Pins.hh"
#include "hw/interfaces/drivers/CanManager.hh"
#include "hw/interfaces/drivers/Mcp2515.hh"
#include "hw/interfaces/pico/AdcPico.hh"
#include "hw/interfaces/pico/SpiPico.hh"
#include "hw/sensors/Battery.hh"
#include "utils/time/PicoClock.hh"
#include "utils/time/Time.hh"

namespace {
constexpr bool IS_SENSOR = false;
}

int main() {
  stdio_usb_init();

  gl::utils::PicoClock clock;
  clock.wait(gl::utils::Time::sec(4));
  std::cout << "starting" << std::endl;
  spi_inst_t* spiInst = SPI_INSTANCE(gl::hw::ACTUATOR_SPI_INSTANCE_NR);
  gl::hw::SpiPico spi =
      IS_SENSOR ? gl::hw::SpiPico(spiInst, gl::hw::PinGpioSensor::SPI_MISO, gl::hw::PinGpioSensor::SPI_MOSI,
                                  gl::hw::PinGpioSensor::SPI_SCK, gl::hw::PinGpioSensor::SPI_CS)
                : gl::hw::SpiPico(spiInst, gl::hw::PinGpioActuator::SPI_MISO, gl::hw::PinGpioActuator::SPI_MOSI,
                                  gl::hw::PinGpioActuator::SPI_SCK, gl::hw::PinGpioActuator::SPI_CS_CAN);
  const gl::hw::CanId canId = IS_SENSOR ? gl::hw::CanId::SENSOR_BOARD : gl::hw::CanId::ACTUATOR_BOARD;
  const std::vector<gl::hw::CanId> subscriptions = IS_SENSOR ? std::vector<gl::hw::CanId>{gl::hw::CanId::ACTUATOR_BOARD}
                                                             : std::vector<gl::hw::CanId>{gl::hw::CanId::SENSOR_BOARD};
  gl::hw::CanManager canManager(std::make_unique<gl::hw::Mcp2515>(spi, clock, canId, subscriptions));

  if (IS_SENSOR) {
    gl::hw::AdcPico adcBat(gl::hw::PinAnalogSensor::BATTERY1);
    gl::hw::Battery batSensor(adcBat, clock);
    std::optional<gl::msg::Battery> batMsg;
    while (true) {
      batMsg = batSensor.getReading();
      if (!batMsg.has_value()) {
        std::cout << "Could not read battery" << std::endl;
        continue;
      }
      const std::vector<std::byte> serializedMsg = gl::msg::serialize(*batMsg, 2, clock.now());
      canManager.writeMsg(serializedMsg);
      const gl::utils::Time waitStart = clock.now();
      while (clock.now() < waitStart + gl::utils::Time::sec(3)) {
        canManager.loop();
        clock.wait(gl::utils::Time::msec(100));
      }
    }
  } else {
    while (true) {
      std::optional<gl::hw::CanManager::CanMsg> msg;
      while (!msg.has_value()) {
        msg = canManager.readMsg();
        canManager.loop();
        clock.wait(gl::utils::Time::msec(10));
      }
      std::cout << "Received data from " << static_cast<uint32_t>(msg->canId) << std::endl;
      const std::optional<gl::msg::Header> header = gl::msg::deserializeHeader(msg->data);
      if (!header.has_value()) {
        std::cout << "Decoding header failed" << std::endl;
        continue;
      }
      const std::optional<gl::msg::Battery> batMsg = gl::msg::deserializeMsg<gl::msg::Battery>(msg->data, *header);
      if (!batMsg.has_value()) {
        std::cout << "failed to deserialize message" << std::endl;
        continue;
      }
      std::cout << "Message Volatage" << batMsg->voltage_v << " at time: " << header->timestamp_us << std::endl;
    }
  }
  return 0;
}
