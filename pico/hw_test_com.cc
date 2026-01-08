#include <cstdint>
#include <iostream>
#include "pico/stdlib.h"

#include "hw/Pins.hh"
#include "hw/interfaces/drivers/CanManager.hh"
#include "hw/interfaces/drivers/Mcp2515.hh"
#include "hw/interfaces/pico/SpiPico.hh"
#include "utils/time/PicoClock.hh"
#include "utils/time/Time.hh"

namespace {
constexpr bool IS_SENSOR = true;
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
    std::vector<std::byte> sendData;
    for (uint32_t i = 0; i < 32; i++) {
      sendData.push_back(static_cast<std::byte>(i));
    }
    while (true) {
      canManager.writeMsg(sendData);
      const gl::utils::Time waitStart = clock.now();
      while (clock.now() < waitStart + gl::utils::Time::sec(3)) {
        canManager.loop();
        clock.wait(gl::utils::Time::msec(100));
      }
    }
  } else {
    gl::hw::Mcp2515 mcp2515(spi, clock, canId, subscriptions);
    while (true) {
      std::optional<gl::hw::CanManager::CanMsg> msg;
      while (!msg.has_value()) {
        msg = canManager.readMsg();
        canManager.loop();
        clock.wait(gl::utils::Time::msec(10));
      }
      std::cout << "Received " << msg->data.size() << " bytes from " << static_cast<uint32_t>(msg->canId) << std::endl;
      std::cout << "data: ";
      for (uint32_t i = 0; i < msg->data.size(); i++) {
        std::cout << static_cast<uint32_t>(msg->data[i]) << " ";
      }
      std::cout << std::endl;
    }
  }
  return 0;
}
