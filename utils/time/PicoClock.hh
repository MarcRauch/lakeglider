#ifndef GL_UTILS_TIME_PICOTCLOCK_H_
#define GL_UTILS_TIME_PICOTCLOCK_H_

#include "utils/time/Time.hh"
#include "utils/time/IClock.hh"

namespace gl::utils {

class PicoClock : public IClock {
 public:
  Time now() const override;

  void wait(Time duration) const override;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_PICOCLOCK_H_