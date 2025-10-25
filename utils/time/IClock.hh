#ifndef GL_UTILS_TIME_ICLOCK_H_
#define GL_UTILS_TIME_ICLOCK_H_

#include "utils/time/Time.hh"

namespace gl::utils {

class IClock {
 public:
  /**
   * Gets the current system time
   * @returns Time with the current time
   */
  virtual Time now() const = 0;

  /**
   * Sleeps for a specified time
   * @param[in] duration Time of sleeping
   */
  virtual void wait(Time duration) const = 0;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_ICLOCK_H_