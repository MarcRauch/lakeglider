#ifndef GL_UTILS_TIME_Time_H_
#define GL_UTILS_TIME_Time_H_

#include <stdint.h>

namespace gl::utils {

/**
Saving time durations and timestamps. Saves at a microsecond resolution
*/
class Time {
 public:
  /**
   * Constructs a Time object.
   * @param[in] time_us Time to be set in micro seconds
   * @returns Time object containing the specified time
   */
  Time(uint64_t time_us = 0) : time_us(time_us){};

  /**
   * Constructs a Time from a given time in seconds of any type
   * @param[in] time_s Time to be set in seconds
   * @returns Time object containing the specified time
   */
  template <typename T>
  static Time sec(T time_s) {
    return Time(static_cast<uint32_t>(time_s * 1e6));
  };

  /**
   * Constructs a Time from a given time in milliseconds of any type
   * @param[in] time_ms Time to be set in milliseconds
   * @returns Time object containing the specified time
   */
  template <typename T>
  static Time msec(T time_ms) {
    return Time(static_cast<uint32_t>(time_ms * 1e3));
  };

  /**
   * Constructs a Time from a given time in microseconds of any type
   * @param[in] time_us Time to be set in microseconds
   * @returns Time object containing the specified time
   */
  template <typename T>
  static Time usec(T time_us) {
    return Time(static_cast<uint32_t>(time_us));
  };

  /**
   * Converts the time to seconds
   * @returns Time in seconds converted to the given type
   */
  template <typename T>
  T sec() const {
    return static_cast<T>(time_us / 1e6) + (static_cast<T>(time_us % 1000000) / static_cast<T>(1e6));
  };

  /**
   * Converts the time to milliseconds
   * @returns Time in milliseconds converted to the given type
   */
  template <typename T>
  T msec() const {
    return static_cast<T>(time_us / 1e3) + (static_cast<T>(time_us % 1000) / static_cast<T>(1e3));
  };

  /**
   * Converts the time to microseconds
   * @returns Time in microseconds converted to the given type
   */
  template <typename T>
  T usec() const {
    return static_cast<T>(time_us);
  };

  /**
   * Overloads the + operator
   * @param[in] rhs timestamp to be added
   * @returns sum of the two timestamps
   */
  Time operator+(const Time& rhs) const { return Time::usec(time_us + rhs.usec<uint64_t>()); }

  /**
   * Overloads the - operator
   * @param[in] rhs timestamp to be subtracted
   * @returns difference of the two timestamps
   */
  Time operator-(const Time& rhs) const { return Time::usec(time_us - rhs.usec<uint64_t>()); }

  /**
   * Overloads the < operator
   * @param[in] rhs timestamp to be compared
   * @returns true if the timestamp to be compared is larger
   */
  bool operator<(const Time& rhs) const { return time_us < rhs.usec<uint64_t>(); }

  /**
   * Overloads the += operator
   * @param[in] rhs the time to be added
   * @returns the value with rhs added to it
   */
  Time& operator+=(const Time& rhs) {
    this->time_us += rhs.usec<int64_t>();
    return *this;
  }

 private:
  int64_t time_us;
};
}  // namespace gl::time

#endif  // GL_UTILS_TIME_Time_H_
