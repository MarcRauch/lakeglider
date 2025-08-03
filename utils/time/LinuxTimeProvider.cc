#include "utils/time/LinuxTimeProvider.hh"

#include <chrono>
#include <thread>

namespace gl::utils {
GlTime LinuxTimeProvider::now() const {
  const auto now = std::chrono::system_clock::now();
  uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
  return GlTime::usec(now_us);
}

void LinuxTimeProvider::wait(GlTime duration) const {
  std::this_thread::sleep_for(std::chrono::microseconds(duration.usec<uint64_t>()));
}
}  // namespace gl::utils