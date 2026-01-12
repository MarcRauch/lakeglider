
#ifndef GL_UTILS_LOGGING_LOGGER_H
#define GL_HW_INTERFACES_ISPI_H_

#include <string>

#include "spdlog/spdlog.h"

namespace gl::utils::log {
/**
 * Creates a color logger with a given name
 * @param[in] name Name of logger to be displayed
 * @returns Created logger
 */
std::unique_ptr<spdlog::logger> createLogger(const std::string& name);
}  // namespace gl::utils::log

#endif  // GL_HW_INTERFACES_ISPI_H
