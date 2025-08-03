#ifndef GL_UTILS_TIME_ITIMEPROVIDER_H_
#define GL_UTILS_TIME_ITIMEPROVIDER_H_

#include "utils/time/GlTime.hh"

namespace gl::utils {

class ITimeProvider {
 public:
  /**
   * Gets the current system time
   * @returns GlTime with the current time
   */
  virtual GlTime now() const = 0;

  /**
   * Sleeps for a specified time
   * @param[in] duration Time of sleeping
   */
  virtual void wait(GlTime duration) const = 0;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_ITIMEPROVIDER_H_