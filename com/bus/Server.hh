#ifndef GL_COM_BUS_SERVER_H_
#define GL_COM_BUS_SERVER_H_

#include <asio.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "com/msg/Msg.hh"
#include "com/msg/Subscriptions.hh"
#include "com/msg/Type.hh"
#include "utils/logging/logger.hh"

namespace gl::msg {
class Server {
 public:
  /**
   * Creates server
   * @param[in] ioContext context to be used for the server. Needs to be run after the server is created
   * @param[in] logger logger used to print messages
   * @returns Server object
   */
  Server(asio::io_context& ioContext, std::shared_ptr<spdlog::logger> logger);

  /**
   * Starts the server on a specified port. Does not set the ioContext to running tho.
   * @param[in] port port to start the server on
   * @returns true if start was successful
   */
  bool start(uint32_t port);

 private:
  /**
   * Class handeling a connection to a client. Interacts with the server for client management
   */
  class Connection : public std::enable_shared_from_this<Connection> {
   public:
    Connection(asio::ip::tcp::socket socket, Server& server, std::shared_ptr<spdlog::logger> logger);
    void start();
    void sendMessage(const gl::msg::Msg& msg);

   private:
    void recvHeader();
    void recvMsg(std::shared_ptr<std::vector<std::byte>> headerVec);

    std::string name;
    asio::ip::tcp::socket socket;
    Server& server;
    std::shared_ptr<spdlog::logger> logger;
  };

  void addClient(const std::string& name, std::shared_ptr<Connection> connection, Subscriptions subscriptionMsg);
  void deleteClient(const std::string& name);
  void broadcastMsg(const gl::msg::Msg& msg, const std::string& senderName);
  void acceptConnection();

  std::shared_ptr<spdlog::logger> logger;
  std::mutex mutex;

  std::unordered_map<gl::msg::Type, std::vector<std::string>> subscriptions;
  std::unordered_map<std::string, std::shared_ptr<Connection>> connectionMap;
  asio::ip::tcp::acceptor acceptor;
};
}  // namespace gl::msg

#endif  // GL_COM_BUS_SERVER_H_
