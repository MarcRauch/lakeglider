#include "utils/time/LoopTimer.hh"

namespace gl::utils {
LoopTimer::LoopTimer(const IClock& clock, Time loopTime) : clock(clock), loopTime(loopTime) {}

void LoopTimer::wait() {
  if (!lastTime.has_value()) {
    lastTime = clock.now();
    return;
  }
  const Time now = clock.now();
  Time nextTime = *lastTime + loopTime;
  if (now > nextTime) {
    nextTime = now;
  }
  const Time timeRemaining = nextTime - now;
  clock.wait(timeRemaining);
  lastTime = nextTime;
}

}  // namespace gl::utils
