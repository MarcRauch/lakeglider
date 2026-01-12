#include "utils/logging/logger.hh"

#include "spdlog/sinks/stdout_color_sinks.h"

namespace gl::utils::log {
std::unique_ptr<spdlog::logger> createLogger(const std::string& name) {
  auto console = std::make_unique<spdlog::sinks::stdout_color_sink_mt>();
  auto logger = std::make_unique<spdlog::logger>(name, std::move(console));

  logger->set_level(spdlog::level::debug);
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
  return logger;
}
}  // namespace gl::utils::log
