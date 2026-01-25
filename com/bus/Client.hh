
#ifndef GL_COM_BUS_CLIENT_H_
#define GL_COM_BUS_CLIENT_H_

#include <asio.hpp>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "com/msg/Msg.hh"
#include "com/msg/Subscriptions.hh"
#include "utils/logging/logger.hh"
#include "utils/time/LinuxClock.hh"

namespace gl::msg {
class Client {
 public:
  /**
   * Create bus Client, subscribing to the given types
   * @param[in] name The name of the client. Needs to be unique on the bus
   * @param[in] subscriptionVec List of types to subscribe to
   * @param[in] logger logger to print messages to
   * @returns Client
   */
  Client(const std::string& name, const std::vector<Type>& subscriptionVec, std::shared_ptr<spdlog::logger> logger);

  /**
   * Disconnects from server and shuts down client
   */
  ~Client();

  /**
   * Connects to a server
   * @param[in] server IP address of the server
   * @param[in] port port to connect to
   * @returns true on success
   */
  bool connect(const std::string& server, uint32_t port);

  /**
   * Sends a message to the server
   * @param[in] msg Message to be sent
   */
  void sendMsg(const gl::msg::Msg& msg);

  /**
   * Blocks and waits for a message on the bus
   * @returns The read message or nullopt if reading failed
   */
  std::optional<gl::msg::Msg> readMsg();

 private:
  void recvHeader();
  void recvMsg(std::shared_ptr<std::vector<std::byte>> headerVec);

  const std::string name;
  bool connected = false;
  Subscriptions subscriptions;

  asio::io_context ioContext;
  asio::ip::tcp::socket socket;
  std::thread networkThread;

  std::mutex queueMutex;
  std::condition_variable queueCondition;
  std::deque<gl::msg::Msg> receivedMessages;

  std::shared_ptr<spdlog::logger> logger;
  utils::LinuxClock clock;
};
}  // namespace gl::msg

#endif  // GL_COM_BUS_CLIENT_H_
