#ifndef GL_UTILS_TIME_LINUXCLOCK_H_
#define GL_UTILS_TIME_LINUXCLOCK_H_

#include "utils/time/Time.hh"
#include "utils/time/IClock.hh"

namespace gl::utils {

class LinuxClock : public IClock {
 public:
  Time now() const override;

  void wait(Time duration) const override;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_LINUXCLOCK_H_