#include <gflags/gflags.h>
#include <asio.hpp>

#include <memory>

#include "com/bus/Server.hh"
#include "utils/logging/logger.hh"

DEFINE_uint32(port, 1234, "Port to run server on");

namespace {
auto logger = gl::utils::log::createLogger("bus server");
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  asio::io_context ioContext;

  gl::msg::Server server(ioContext, logger);

  if (!server.start(FLAGS_port)) {
    logger->error("Failed to start server");
    return 1;
  }

  logger->info("Starting server on port {}", FLAGS_port);
  ioContext.run();

  return 0;
}
