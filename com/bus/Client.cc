#include "com/bus/Client.hh"

#include <cstring>
#include <memory>
#include <mutex>

#include "com/msg/Header.hh"
#include "com/msg/Msg.hh"
#include "com/msg/Subscriptions.hh"
#include "com/msg/Type.hh"

namespace gl::msg {
Client::Client(const std::string& name, const std::vector<Type>& subscriptionVec,
               std::shared_ptr<spdlog::logger> logger)
    : name(name), socket(ioContext), logger(logger) {
  subscriptions.nSubscriptions = subscriptionVec.size();
  std::copy(subscriptionVec.begin(), subscriptionVec.end(), subscriptions.subscriptions);
  std::memset(subscriptions.name, 0, sizeof(subscriptions.name));
  std::copy(name.begin(), name.end(), subscriptions.name);
}

Client::~Client() {
  ioContext.stop();
  networkThread.join();
};

bool Client::connect(const std::string& address, uint32_t port) {
  asio::ip::tcp::resolver resolver(ioContext);

  asio::error_code errorCode;
  const asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(address, std::to_string(port), errorCode);
  if (errorCode) {
    logger->error("Failed to resolve server: {}", errorCode.message());
    return false;
  }
  asio::connect(socket, endpoints, errorCode);
  if (errorCode) {
    logger->error("Failed to connect to server: {}", errorCode.message());
    return false;
  }

  recvHeader();
  networkThread = std::thread([this]() { ioContext.run(); });

  gl::msg::Msg subscriptionsMsg(subscriptions, 0, clock.now());
  sendMsg(subscriptionsMsg);
  connected = true;
  return true;
}

void Client::sendMsg(const gl::msg::Msg& msg) {
  const std::shared_ptr<gl::msg::Msg> msgPtr = std::make_shared<gl::msg::Msg>(msg);
  asio::async_write(socket, asio::buffer(msgPtr->getSerializedMsg()),
                    [msgPtr, this](asio::error_code errorCode, std::size_t nBytes) {
                      if (errorCode) {
                        logger->error("Failed to send message: {}", errorCode.message());
                      }
                    });
}

std::optional<gl::msg::Msg> Client::readMsg(bool waitBlocking) {
  std::unique_lock<std::mutex> lock(queueMutex);
  if (!connected || (!waitBlocking && receivedMessages.empty())) {
    return std::nullopt;
  }
  queueCondition.wait(lock, [this] { return !receivedMessages.empty(); });

  gl::msg::Msg msg = std::move(receivedMessages.front());
  receivedMessages.pop_front();
  return msg;
}

void Client::recvHeader() {
  std::shared_ptr<std::vector<std::byte>> headerVec = std::make_shared<std::vector<std::byte>>(sizeof(gl::msg::Header));
  asio::async_read(socket, asio::buffer(*headerVec), [this, headerVec](asio::error_code errorCode, std::size_t nBytes) {
    if (errorCode) {
      logger->error("Failed to read header: {}", errorCode.message());
      connected = false;
      return;
    }

    recvMsg(headerVec);
  });
}

void Client::recvMsg(std::shared_ptr<std::vector<std::byte>> headerVec) {
  const uint32_t bytesRemaining = reinterpret_cast<const gl::msg::Header*>(headerVec->data())->msgLength + 2;
  const uint32_t msgSize = sizeof(gl::msg::Header) + bytesRemaining;
  headerVec->resize(msgSize);

  asio::async_read(socket, asio::buffer(headerVec->data() + sizeof(gl::msg::Header), bytesRemaining),
                   [this, headerVec](asio::error_code errorCode, std::size_t nBytes) {
                     if (errorCode) {
                       logger->error("Failed to read message body: {}", errorCode.message());
                       return;
                     }

                     const std::optional<Msg> msg = Msg::fromBytes(*headerVec);
                     if (!msg.has_value()) {
                       logger->error("failed to decode received message");
                     } else {
                       std::scoped_lock queueLock(queueMutex);
                       receivedMessages.push_back(std::move(*msg));
                       queueCondition.notify_one();
                     }
                     recvHeader();
                   });
}

}  // namespace gl::msg
