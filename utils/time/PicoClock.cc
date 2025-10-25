#include "utils/time/PicoClock.hh"

#include <pico/stdlib.h>

namespace gl::utils {
Time PicoClock::now() const {
  return Time::usec(time_us_64());
}

void PicoClock::wait(Time duration) const {
  sleep_us(duration.usec<uint64_t>());
}
}  // namespace gl::utils