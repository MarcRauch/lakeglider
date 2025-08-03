#include "utils/time/PicoTimeProvider.hh"

#include <pico/stdlib.h>

namespace gl::utils {
GlTime PicoTimeProvider::now() const {
  return GlTime::usec(time_us_64());
}

void PicoTimeProvider::wait(GlTime duration) const {
  sleep_us(duration.usec<uint64_t>());
}
}  // namespace gl::utils