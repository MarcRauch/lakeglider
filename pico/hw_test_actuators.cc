#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "pico/stdlib.h"

#include "hw/actuators/EqiUartMg1.hh"
#include "hw/actuators/Tmc5160.hh"
#include "hw/actuators/Valve.hh"
#include "hw/interfaces/pico/AdcPico.hh"
#include "hw/interfaces/pico/GpioPico.hh"
#include "hw/interfaces/pico/SpiPico.hh"
#include "hw/interfaces/pico/UartPico.hh"
#include "hw/sensors/Potentiometer.hh"
#include "utils/time/PicoClock.hh"

int main() {
  gl::utils::PicoClock clock;
  stdio_usb_init();
  gpio_init(static_cast<uint8_t>(gl::hw::PinGpioActuator::LED_RED));
  gpio_init(static_cast<uint8_t>(gl::hw::PinGpioActuator::LED_GREEEN));
  gpio_set_dir(static_cast<uint8_t>(gl::hw::PinGpioActuator::LED_RED), GPIO_OUT);
  gpio_set_dir(static_cast<uint8_t>(gl::hw::PinGpioActuator::LED_GREEEN), GPIO_OUT);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioActuator::LED_RED), true);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioActuator::LED_GREEEN), false);
  clock.wait(gl::utils::Time::sec(4.));
  std::cout << "Starting..." << std::endl;

  // init HW
  gl::hw::AdcPico adcPot(gl::hw::PinAnalogActuator::PUMP_MEAS);
  uart_inst* uartInst = UART_INSTANCE(gl::hw::ACTUATOR_UART_INSTANCE_NR);
  spi_inst_t* spiInst = SPI_INSTANCE(gl::hw::ACTUATOR_SPI_INSTANCE_NR);
  gl::hw::UartPico uartPump(uartInst, gl::hw::PinGpioActuator::PUMP_TX, gl::hw::PinGpioActuator::PUMP_RX, 115200);
  gl::hw::SpiPico spiMot1(spiInst, gl::hw::PinGpioActuator::SPI_MISO, gl::hw::PinGpioActuator::SPI_MOSI,
                          gl::hw::PinGpioActuator::SPI_SCK, gl::hw::PinGpioActuator::SPI_CS_MOT1);
  gl::hw::SpiPico spiMot2(spiInst, gl::hw::PinGpioActuator::SPI_MISO, gl::hw::PinGpioActuator::SPI_MOSI,
                          gl::hw::PinGpioActuator::SPI_SCK, gl::hw::PinGpioActuator::SPI_CS_MOT2);
  gl::hw::GpioPico gpioValve(gl::hw::PinGpioActuator::VALVE, true);
  gl::hw::Valve valve(&gpioValve);
  gl::hw::Tmc5160::Config config1 = {.maxRotations = 15.};
  gl::hw::Tmc5160 motor1(&spiMot1, &clock, config1);
  gl::hw::Tmc5160::Config config2 = {.maxRotations = 0.2};
  gl::hw::Tmc5160 motor2(&spiMot2, &clock, config2);
  gl::hw::Potentiometer potentiometer(&adcPot, 1., 2.);
  gl::hw::EqiUartMg1 pump(&uartPump);

  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_RED), false);
  gpio_put(static_cast<uint8_t>(gl::hw::PinGpioSensor::LED_GREEEN), true);
  std::cout << "Hardware test: To start component type motor[1, 2] position, valve [open, close], pump [start, stop] ";
  std::cout << "speed [in, out], potentiometer." << std::endl;

  while (true) {
    std::vector<std::string> command;
    std::string commandLine;
    std::getline(std::cin, commandLine);
    std::istringstream iss(commandLine);
    std::string commandPart;
    while (iss >> commandPart) {
      auto removeGarbage =
          std::remove_if(commandPart.begin(), commandPart.end(), [](unsigned char c) { return !std::isprint(c); });
      commandPart.erase(removeGarbage, commandPart.end());
      command.push_back(commandPart);
    }

    // Match to commands
    if (command[0].starts_with("motor")) {
      if (command.size() != 2) {
        std::cout << "Proper use: motor[1, 2] position" << std::endl;
        continue;
      }
      const uint32_t motorNr = std::stoi(command[0].substr(5));
      if (motorNr != 1 && motorNr != 2) {
        std::cout << "Motor id must be 1 or 2. Currently is " << motorNr << std::endl;
        continue;
      }
      if (motorNr == 1) {
        if (command[1] == "home") {
          std::cout << "before home" << std::endl;
          motor1.home();
          std::cout << "Homing motor " << motorNr << std::endl;
        } else {
          const double pos = std::stod(command[1]);
          motor1.moveTo(pos);
          std::cout << "Set motor " << motorNr << " position to " << pos << std::endl;
        }

      } else if (motorNr == 2) {
        if (command[1] == "home") {
          motor2.home();
          std::cout << "Homing motor " << motorNr << std::endl;
        } else {
          const double pos = std::stod(command[1]);
          motor2.moveTo(pos);
          std::cout << "Set motor " << motorNr << " position to " << pos << std::endl;
        }
      }
    } else if (command[0] == "valve") {
      if (command.size() != 2 || (command[1] != "open" && command[1] != "close")) {
        std::cout << "Proper use: valve [open, close]" << std::endl;
        continue;
      }
      if (command[1] == "open") {
        valve.open();
      } else if (command[1] == "close") {
        valve.close();
      }
      std::cout << "Performed " << command[1] << " of the Valve" << std::endl;
    } else if (command[0] == "pump") {
      if (command.size() < 2 || (command[1] != "start" && command[1] != "stop")) {
        std::cout << "Proper use: pump [start, stop]" << std::endl;
        continue;
      }
      if (command[1] == "start") {
        if (command.size() != 4 || (command[3] != "in" && command[3] != "out")) {
          std::cout << "Proper use: pump [start, stop] speed [in, out]" << std::endl;
          continue;
        }
        if (command[3] == "in") {
          pump.setDirection(true);
        } else if (command[3] == "out") {
          pump.setDirection(false);
        }
        const double speed = std::stod(command[2]);
        std::cout << "Starting pump with speed " << speed << std::endl;
        pump.setSpeed(speed);
        pump.start();
      } else if (command[1] == "stop") {
        pump.stop();
        std::cout << "Stopped pump" << std::endl;
      }
    } else if (command[0] == "potentiometer") {
      std::cout << "Reading potentiometer: " << potentiometer.read() << std::endl;
    } else {
      std::cout << "Unknown command " << command[0] << std::endl;
    }
  }
}
