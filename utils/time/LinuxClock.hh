#ifndef GL_UTILS_TIME_LINUXCLOCK_H_
#define GL_UTILS_TIME_LINUXCLOCK_H_

#include "utils/time/GlTime.hh"
#include "utils/time/IClock.hh"

namespace gl::utils {

class LinuxClock : public IClock {
 public:
  GlTime now() const override;

  void wait(GlTime duration) const override;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_LINUXCLOCK_H_