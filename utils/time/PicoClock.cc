#include "utils/time/PicoClock.hh"

#include <pico/stdlib.h>

namespace gl::utils {
GlTime PicoClock::now() const {
  return GlTime::usec(time_us_64());
}

void PicoClock::wait(GlTime duration) const {
  sleep_us(duration.usec<uint64_t>());
}
}  // namespace gl::utils