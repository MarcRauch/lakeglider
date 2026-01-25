#include "com/bus/Server.hh"
#include <memory>
#include "com/msg/Header.hh"
#include "com/msg/Subscriptions.hh"
#include "com/msg/Type.hh"

namespace gl::msg {

Server::Connection::Connection(asio::ip::tcp::socket socket, Server& server, std::shared_ptr<spdlog::logger> logger)
    : socket(std::move(socket)), server(server), logger(logger) {}

void Server::Connection::start() {
  recvHeader();
}

void Server::Connection::sendMessage(const gl::msg::Msg& msg) {
  const std::shared_ptr<Server::Connection> self = shared_from_this();
  const std::shared_ptr<gl::msg::Msg> msgPtr = std::make_shared<gl::msg::Msg>(msg);
  asio::async_write(socket, asio::buffer(msgPtr->getSerializedMsg()),
                    [this, self, msgPtr](asio::error_code errorCode, std::size_t nBytes) {
                      if (errorCode) {
                        logger->error("Failed to send message: {}", errorCode.message());
                      }
                    });
}

void Server::Connection::recvHeader() {
  const std::shared_ptr<Server::Connection> self = shared_from_this();
  std::shared_ptr<std::vector<std::byte>> headerVec = std::make_shared<std::vector<std::byte>>(sizeof(gl::msg::Header));
  asio::async_read(socket, asio::buffer(*headerVec),
                   [this, self, headerVec](asio::error_code errorCode, std::size_t nBytes) {
                     if (errorCode) {
                       logger->error("Failed to read message: {}", errorCode.message());
                       server.deleteClient(name);
                       return;
                     }

                     recvMsg(headerVec);
                   });
}

void Server::Connection::recvMsg(std::shared_ptr<std::vector<std::byte>> msgVec) {
  const std::shared_ptr<Server::Connection> self = shared_from_this();
  const uint32_t bytesRemaining = reinterpret_cast<const gl::msg::Header*>(msgVec->data())->msgLength + 2;
  const uint32_t msgSize = sizeof(gl::msg::Header) + bytesRemaining;
  msgVec->resize(msgSize);

  asio::async_read(socket, asio::buffer(msgVec->data() + sizeof(gl::msg::Header), bytesRemaining),
                   [this, self, msgVec](asio::error_code errorCode, std::size_t nBytes) {
                     if (errorCode) {
                       logger->error("Failed to read message body: {}", errorCode.message());
                       return;
                     }

                     const std::optional<gl::msg::Msg> msg = gl::msg::Msg::fromBytes(*msgVec);
                     if (!msg.has_value()) {
                       logger->error("Failed to decode message");
                       return;
                     }

                     if (msg->getHeader().type == Type::Subscriptions) {
                       const std::optional<Subscriptions> subscriptionMsg = msg->getMsg<Subscriptions>();
                       if (!subscriptionMsg.has_value()) {
                         logger->error("Could not decode subscription message");
                         return;
                       }
                       name = subscriptionMsg->name;
                       server.addClient(name, self, *subscriptionMsg);
                     }

                     server.broadcastMsg(*msg, name);

                     recvHeader();
                   });
}

Server::Server(asio::io_context& ioContext, std::shared_ptr<spdlog::logger> logger)
    : logger(logger), acceptor(ioContext) {
  acceptor = asio::ip::tcp::acceptor(ioContext);
}

bool Server::start(uint32_t port) {
  asio::error_code errorCode;
  acceptor.open(asio::ip::tcp::v4(), errorCode);
  if (errorCode) {
    logger->info("Failed to open ipV4 server with: {}", errorCode.message());
    return false;
  }
  acceptor.bind(asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port), errorCode);
  if (errorCode) {
    logger->info("Failed to bind to port {} with: {}", port, errorCode.message());
    return false;
  }
  acceptor.listen(asio::socket_base::max_listen_connections, errorCode);
  if (errorCode) {
    logger->info("Failed to start listening with: {}", errorCode.message());
    return false;
  }

  acceptConnection();

  return true;
}

void Server::addClient(const std::string& name, std::shared_ptr<Connection> connection, Subscriptions subscriptionMsg) {
  logger->info("Adding client {}", name);
  connectionMap[name] = connection;

  for (uint32_t i = 0; i < subscriptionMsg.nSubscriptions; i++) {
    const Type currSubscription = subscriptionMsg.subscriptions[i];
    if (!subscriptions.contains(currSubscription)) {
      subscriptions[currSubscription] = {};
    }
    subscriptions[currSubscription].push_back(name);
  }
}

void Server::deleteClient(const std::string& name) {
  connectionMap.erase(name);
  for (auto& subEntry : subscriptions) {
    std::erase(subEntry.second, name);
  }
}

void Server::broadcastMsg(const gl::msg::Msg& msg, const std::string& senderName) {
  const Type type = msg.getHeader().type;
  for (auto& name : subscriptions[type]) {
    if (name != senderName) {
      connectionMap[name]->sendMessage(msg);
    }
  }
}

void Server::acceptConnection() {
  acceptor.async_accept([this](asio::error_code errorCode, asio::ip::tcp::socket socket) {
    if (!errorCode) {
      std::shared_ptr<Connection> connection = std::make_shared<Connection>(std::move(socket), *this, logger);
      connection->start();
    } else {
      logger->error("Failed to accept connection: {}", errorCode.message());
    }
    acceptConnection();
  });
}
}  // namespace gl::msg
