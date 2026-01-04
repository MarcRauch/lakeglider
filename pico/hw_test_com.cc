#include <array>
#include <cstdint>
#include <iostream>
#include "pico/stdlib.h"

#include "hw/Pins.hh"
#include "hw/interfaces/drivers/Mcp2515.hh"
#include "hw/interfaces/pico/SpiPico.hh"
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
  gl::hw::Mcp2515 mcp2515(spi, clock, canId, subscriptions);

  if (IS_SENSOR) {
    while (true) {
      const std::array<std::byte, 8> data = {std::byte(0x01), std::byte(0x02), std::byte(0x03)};
      mcp2515.send(data, 3);
      clock.wait(gl::utils::Time::sec(3.));
    }
  } else {
    while (true) {
      std::array<std::byte, 8> recv;
      auto [numBytes, canId] = mcp2515.read(recv);
      while (numBytes == 0) {
        clock.wait(gl::utils::Time::sec(1));
        std::tie(numBytes, canId) = mcp2515.read(recv);
      }
      std::cout << "Received " << static_cast<uint32_t>(numBytes) << " bytes from " << static_cast<uint32_t>(canId)
                << std::endl;
      std::cout << "data: ";
      for (uint32_t i = 0; i < numBytes; i++) {
        std::cout << static_cast<uint32_t>(recv[i]) << " ";
      }
      std::cout << std::endl;
    }
  }
  return 0;
}
