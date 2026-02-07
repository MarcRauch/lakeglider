
#ifndef GL_UTILS_TIME_LOOPTIMER_H_
#define GL_UTILS_TIME_LOOPTIMER_H_

#include <optional>

#include "utils/time/IClock.hh"
#include "utils/time/Time.hh"

namespace gl::utils {

class LoopTimer {
 public:
  /**
   * Creates a loop timer object, used to execute code in a loop in a specified interval
   * @param[in] clock Clock used for the timing
   * @param[in] loopTime the period time at which the loop is run
   * @returns a looptimer object
   */
  LoopTimer(const IClock& clock, Time loopTime);

  /**
   * Call at the beginning or end of the loop. Waits until the next iteration should be run
   */
  void wait();

 private:
  const IClock& clock;
  const Time loopTime;
  std::optional<Time> lastTime;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_LOOPTIMER_H_
