#ifndef GL_UTILS_TIME_TIMESYNCHRONIZER_H_
#define GL_UTILS_TIME_TIMESYNCHRONIZER_H_

#include <optional>

#include "utils/time/Time.hh"

namespace gl::utils {
class TimeSynchronizer {
 public:
  /**
   * Creates time synchronizer object
   */
  TimeSynchronizer() = default;

  /**
   * Adds a timestamp pair to calculate the offset between two sources
   * @param[in] srcTimestamp Local time
   * @param[in] dstTimestamp Time to which we want to synchronize
   */
  void updateTimeSync(Time srcTimestamp, Time dstTimestamp);

  /** Converts timestamp from src to dst
   * @param[in] srcTimestamp Local timestamp to be converted
   * @returns The timestamp synchronized to dst
   */
  Time getSynchedTime(Time srcTimestamp) const;

  /**
   * Checks if timestamp synchronization is active
   * @returns True of synchronization is ready
   */
  bool isSynchronized() const;

 private:
  std::optional<Time> timeDiff;
};
}  // namespace gl::utils

#endif  // GL_UTILS_TIME_Time_H_
