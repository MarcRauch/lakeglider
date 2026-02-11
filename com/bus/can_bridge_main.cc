#include <gflags/gflags.h>

#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "com/bus/Client.hh"
#include "com/msg/Msg.hh"
#include "com/msg/Timesync.hh"
#include "com/msg/Type.hh"
#include "hw/Pins.hh"
#include "hw/interfaces/drivers/CanManager.hh"
#include "hw/interfaces/rpi/SpiRpi.hh"
#include "utils/logging/logger.hh"
#include "utils/time/LinuxClock.hh"
#include "utils/time/LoopTimer.hh"
#include "utils/time/Time.hh"

DEFINE_string(server_address, "localhost", "Address of the server");
DEFINE_uint32(server_port, 1234, "Port to connect to");
DEFINE_string(spi_path, "/dev/spidev0.0", "Path to spi device connected to the CAN interface");

namespace {
constexpr double CAN_LOOP_PERIOD_US = 300;
constexpr double BUS_LOOP_PERIOD_US = 500;

auto logger = gl::utils::log::createLogger("Can Bridge");

/// Reads and writes to CAN bus. communicates with two queues
void canThreadFunc(std::queue<gl::msg::Msg>& recvQueue, std::queue<gl::msg::Msg>& sendQueue, std::mutex& queueMutex,
                   const std::string& spiPath) {
  gl::utils::LinuxClock clock;
  gl::utils::LoopTimer canLoopTimer(clock, gl::utils::Time::usec(CAN_LOOP_PERIOD_US));

  gl::hw::SpiRpi spi(spiPath);
  const gl::hw::CanId canId = gl::hw::CanId::COMPUTE_BOARD;
  const std::vector<gl::hw::CanId> canSubscriptions = {gl::hw::CanId::ACTUATOR_BOARD, gl::hw::CanId::SENSOR_BOARD};
  gl::hw::CanManager canManager(std::make_unique<gl::hw::Mcp2515>(spi, clock, canId, canSubscriptions));

  while (true) {
    canManager.loop();
    std::optional<gl::hw::CanManager::CanMsg> recvMsg = canManager.readMsg();
    if (recvMsg.has_value()) {
      const std::optional<gl::msg::Msg> msg = gl::msg::Msg::fromBytes(std::span(recvMsg->data).first(recvMsg->msgLen));
      if (msg.has_value()) {
        std::unique_lock<std::mutex> lock(queueMutex);
        recvQueue.push(std::move(*msg));
      }
    }

    std::unique_lock<std::mutex> lock(queueMutex);
    if (!sendQueue.empty()) {
      gl::msg::Msg sendMsg = sendQueue.front();
      sendQueue.pop();
      canManager.writeMsg(sendMsg.getSerializedMsg());
    }
    lock.unlock();
    canLoopTimer.wait();
  }
}
}  // namespace

int main(int argc, char* argv[]) {
  const std::vector<gl::msg::Type> subscriptions = {};
  gl::msg::Client client("CanBridge", subscriptions, logger);
  client.connect(FLAGS_server_address, FLAGS_server_port);

  std::mutex canQueueMutex;
  std::queue<gl::msg::Msg> canSendQueue;
  std::queue<gl::msg::Msg> canRecvQueue;
  std::thread canThread(canThreadFunc, std::ref(canRecvQueue), std::ref(canSendQueue), std::ref(canQueueMutex),
                        FLAGS_spi_path);

  gl::utils::LinuxClock clock;
  gl::utils::LoopTimer busLoopTimer(clock, gl::utils::Time::usec(BUS_LOOP_PERIOD_US));

  // TODO: Block instead of timed looping
  while (true) {
    if (const std::optional<gl::msg::Msg> recvMsg = client.readMsg(true)) {
      std::unique_lock<std::mutex> lock(canQueueMutex);
      canSendQueue.push(*recvMsg);
    }

    std::unique_lock<std::mutex> lock(canQueueMutex);
    while (!canRecvQueue.empty()) {
      const gl::msg::Msg recvMsg = canRecvQueue.front();
      if (recvMsg.getHeader().type == gl::msg::Type::TimeSync) {
        if (std::optional<gl::msg::TimeSync> timeSync = recvMsg.getMsg<gl::msg::TimeSync>()) {
          timeSync->recvTimestamp_us = clock.now().usec<uint64_t>();
          canSendQueue.push(gl::msg::Msg(*timeSync, recvMsg.getHeader().deviceId,
                                         gl::utils::Time::usec(recvMsg.getHeader().timestamp_us)));
        }
      }
      canRecvQueue.pop();
      client.sendMsg(recvMsg);
    }
    canQueueMutex.unlock();

    busLoopTimer.wait();
  }

  canThread.join();
  return 0;
}
