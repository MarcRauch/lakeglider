#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>

#include <vector>

#include "com/msg/Msg.hh"
#include "com/msg/Timesync.hh"
#include "com/msg/Type.hh"
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
#include "utils/time/TimeSynchronizer.hh"

namespace {
constexpr double COM_LOOP_PERIOD_US = 250;
constexpr double SENSOR_LOOP_PERIOD_MS = 300;
constexpr double TIMESYNC_PERIOD_S = 5;

queue_t msgQueue;
gl::utils::TimeSynchronizer timeSynchronizer;

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
  gl::utils::Time lastTimeSync(0);
  const gl::utils::Time timeSyncPeriod = gl::utils::Time::sec(TIMESYNC_PERIOD_S);

  gl::msg::Msg msg;
  while (true) {
    // Request timestamp sync
    const gl::utils::Time currTime = clock.now();
    if (currTime >= lastTimeSync + timeSyncPeriod) {
      const gl::msg::TimeSync timeSyncMsg = {.canId = canId, .initialTimestamp_us = currTime.usec<uint64_t>()};
      const gl::msg::Msg serializedMsg(timeSyncMsg, 0, currTime);
      queue_add_blocking(&msgQueue, &serializedMsg);
      lastTimeSync = currTime;
    }

    // Send Message
    while (queue_try_remove(&msgQueue, &msg)) {
      canManager.writeMsg(msg.getSerializedMsg());
    }
    canManager.loop();

    // Update time sync on reply
    if (const std::optional<gl::hw::CanManager::CanMsg> recvMsg = canManager.readMsg()) {
      if (const std::optional<gl::msg::Msg> msg =
              gl::msg::Msg::fromBytes(std::span(recvMsg->data).first(recvMsg->msgLen))) {
        if (msg->getHeader().type == gl::msg::Type::TimeSync) {
          if (const std::optional<gl::msg::TimeSync> timeSyncMsg = msg->getMsg<gl::msg::TimeSync>()) {
            const gl::utils::Time srcTime = 0.5 * (currTime + timeSyncMsg->initialTimestamp_us);
            timeSynchronizer.updateTimeSync(srcTime, timeSyncMsg->recvTimestamp_us);
          }
        }
      }
    }

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

  auto submitMsg = [&](const auto& msg, uint8_t deviceId) {
    if (!timeSynchronizer.isSynchronized()) {
      return;
    }
    const gl::utils::Time syncedTime = timeSynchronizer.getSynchedTime(clock.now());
    const gl::msg::Msg serializedMsg = gl::msg::Msg(msg, deviceId, syncedTime);
    queue_add_blocking(&msgQueue, &serializedMsg);
  };

  // TODO: Consider different rates for different sensors
  gl::utils::LoopTimer loopTimer(clock, gl::utils::Time::msec(SENSOR_LOOP_PERIOD_MS));
  while (true) {
    if (const std::optional<gl::msg::Battery> batMsg1 = bat1.getReading()) {
      submitMsg(*batMsg1, 0);
    }

    if (const std::optional<gl::msg::Battery> batMsg2 = bat2.getReading()) {
      submitMsg(*batMsg2, 1);
    }

    if (const std::optional<gl::msg::Imu> imuMsg = mpu9250.getImuReading()) {
      submitMsg(*imuMsg, 0);
    }

    if (const std::optional<gl::msg::Compass> compassMsg = mpu9250.getCompassReading()) {
      submitMsg(*compassMsg, 0);
    }

    if (const std::optional<gl::msg::Depth> depthMsg = ms5837.loop()) {
      submitMsg(*depthMsg, 0);
    }

    loopTimer.wait();
  }

  return 0;
}
