#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>

#include <vector>

#include "com/msg/Battery.hh"
#include "com/msg/Compass.hh"
#include "com/msg/Depth.hh"
#include "com/msg/Imu.hh"
#include "com/msg/Msg.hh"
#include "hw/Pins.hh"
#include "hw/interfaces/drivers/CanManager.hh"
#include "hw/interfaces/pico/AdcPico.hh"
#include "hw/interfaces/pico/GpioPico.hh"
#include "hw/interfaces/pico/I2cPico.hh"
#include "hw/interfaces/pico/SpiPico.hh"
#include "hw/sensors/Battery.hh"
#include "hw/sensors/Mpu9250.hh"
#include "hw/sensors/Ms5837.hh"
#include "utils/time/LoopTimer.hh"
#include "utils/time/PicoClock.hh"
#include "utils/time/Time.hh"

namespace {
constexpr double COM_LOOP_PERIOD_US = 250;
constexpr double SENSOR_LOOP_PERIOD_MS = 300;

queue_t msgQueue;

// Reads and writes messages to the CAN bus
void comCore() {
  gl::utils::PicoClock clock;
  gl::utils::LoopTimer loopTimer(clock, gl::utils::Time::usec(COM_LOOP_PERIOD_US));
  spi_inst_t* spiInst = SPI_INSTANCE(gl::hw::SENSOR_SPI_INSTANCE_NR);
  gl::hw::SpiPico spi = gl::hw::SpiPico(spiInst, gl::hw::PinGpioSensor::SPI_MISO, gl::hw::PinGpioSensor::SPI_MOSI,
                                        gl::hw::PinGpioSensor::SPI_SCK, gl::hw::PinGpioSensor::SPI_CS);
  const gl::hw::CanId canId = gl::hw::CanId::SENSOR_BOARD;
  const std::vector<gl::hw::CanId> subscriptions = std::vector<gl::hw::CanId>{gl::hw::CanId::COMPUTE_BOARD};
  gl::hw::CanManager canManager(std::make_unique<gl::hw::Mcp2515>(spi, clock, canId, subscriptions));

  gl::msg::Msg msg;
  while (true) {
    while (queue_try_remove(&msgQueue, &msg)) {
      canManager.writeMsg(msg.getSerializedMsg());
    }

    canManager.loop();
    loopTimer.wait();
  }
}
}  // namespace

int main() {
  gl::hw::GpioPico ledRed(gl::hw::PinGpioSensor::LED_RED, true);
  gl::hw::GpioPico ledGreen(gl::hw::PinGpioSensor::LED_GREEEN, true);
  ledRed.setHigh();
  ledGreen.setLow();

  stdio_usb_init();
  gl::utils::PicoClock clock;
  queue_init(&msgQueue, sizeof(gl::msg::Msg), 16);
  multicore_launch_core1(comCore);

  gl::hw::AdcPico adcBat1(gl::hw::PinAnalogSensor::BATTERY1);
  gl::hw::AdcPico adcBat2(gl::hw::PinAnalogSensor::BATTERY2);
  i2c_inst_t* i2cInst = i2c_get_instance(gl::hw::SENSOR_I2C_INSTANCE_NR);
  gl::hw::I2cPico i2c(i2cInst, gl::hw::PinGpioSensor::I2C_SCL, gl::hw::PinGpioSensor::I2C_SDA);

  gl::hw::Battery bat1(adcBat1, clock);
  gl::hw::Battery bat2(adcBat2, clock);
  gl::hw::Ms5837 ms5837(i2c, clock);
  gl::hw::Mpu9250 mpu9250(i2c, clock);

  ms5837.initialize();
  mpu9250.initialize();

  ledRed.setLow();
  ledGreen.setHigh();

  // TODO: Consider different rates for different sensors
  gl::utils::LoopTimer loopTimer(clock, gl::utils::Time::msec(SENSOR_LOOP_PERIOD_MS));
  while (true) {
    const std::optional<gl::msg::Battery> batMsg1 = bat1.getReading();
    if (batMsg1.has_value()) {
      const gl::msg::Msg msg = gl::msg::Msg(*batMsg1, 0, clock.now());
      queue_add_blocking(&msgQueue, &msg);
    }

    const std::optional<gl::msg::Battery> batMsg2 = bat2.getReading();
    if (batMsg2.has_value()) {
      const gl::msg::Msg msg = gl::msg::Msg(*batMsg2, 1, clock.now());
      queue_add_blocking(&msgQueue, &msg);
    }

    const std::optional<gl::msg::Imu> imuMsg = mpu9250.getImuReading();
    if (imuMsg.has_value()) {
      const gl::msg::Msg msg = gl::msg::Msg(*imuMsg, 0, clock.now());
      queue_add_blocking(&msgQueue, &msg);
    }

    const std::optional<gl::msg::Compass> compassMsg = mpu9250.getCompassReading();
    if (compassMsg.has_value()) {
      const gl::msg::Msg msg = gl::msg::Msg(*compassMsg, 0, clock.now());
      queue_add_blocking(&msgQueue, &msg);
    }

    const std::optional<gl::msg::Depth> depthMsg = ms5837.loop();
    if (depthMsg.has_value()) {
      const gl::msg::Msg msg = gl::msg::Msg(*depthMsg, 0, clock.now());
      queue_add_blocking(&msgQueue, &msg);
    }

    loopTimer.wait();
  }

  return 0;
}
