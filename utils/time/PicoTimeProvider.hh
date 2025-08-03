#ifndef GL_UTILS_TIME_PICOTIMEPROVIDER_H_
#define GL_UTILS_TIME_PICOTIMEPROVIDER_H_

#include "utils/time/GlTime.hh"
#include "utils/time/ITimeProvider.hh"

namespace gl::utils {

class PicoTimeProvider : public ITimeProvider {
 public:
  GlTime now() const override;

  void wait(GlTime duration) const override;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_PICOTIMEPROVIDER_H_