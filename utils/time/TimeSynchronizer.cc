#include "utils/time/TimeSynchronizer.hh"

namespace gl::utils {

void TimeSynchronizer::updateTimeSync(Time srcTimestamp, Time dstTimestamp) {
  const Time newTimeDiff = dstTimestamp - srcTimestamp;
  if (!timeDiff.has_value()) {
    timeDiff = newTimeDiff;
  } else {
    *timeDiff = 0.6 * newTimeDiff + 0.4 * *timeDiff;
  }
}

Time TimeSynchronizer::getSynchedTime(Time srcTimestamp) const {
  if (!timeDiff.has_value()) {
    return srcTimestamp;
  }
  return srcTimestamp + *timeDiff;
}

bool TimeSynchronizer::isSynchronized() const {
  return timeDiff.has_value();
}
}  // namespace gl::utils
