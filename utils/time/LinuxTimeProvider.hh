#ifndef GL_UTILS_TIME_LINUXTIMEPROVIDER_H_
#define GL_UTILS_TIME_LINUXTIMEPROVIDER_H_

#include "utils/time/GlTime.hh"
#include "utils/time/ITimeProvider.hh"

namespace gl::utils {

class LinuxTimeProvider : public ITimeProvider {
 public:
  GlTime now() const override;

  void wait(GlTime duration) const override;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_LINUXTIMEPROVIDER_H_