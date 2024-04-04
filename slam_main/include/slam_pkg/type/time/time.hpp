//
// Created by park on 24. 4. 2.
//

#pragma once

#include <chrono>

namespace Slam {

struct Time {
  using Clock = std::chrono::system_clock;

  template <class Duration>
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  using TimePointDefault = std::chrono::time_point<Clock>;

  using Ns = std::chrono::nanoseconds;
  using Ms = std::chrono::milliseconds;
  using Sec = std::chrono::seconds;

  Time() = default;

  Time(TimePoint<Ns> tp) : value(Clock::to_time_t(tp)) {}

  Time(TimePoint<Ms> tp) : value(Clock::to_time_t(tp)) {}

  Time(TimePoint<Sec> tp) : value(Clock::to_time_t(tp)) {}

  Time(std::time_t t) : value(t) {}

  [[nodiscard]] TimePoint<Ns> to_time_point_ns() const {
    return Clock::from_time_t(value);
  }

  [[nodiscard]] TimePoint<Ms> to_time_point_ms() const {
    return std::chrono::time_point_cast<Ms>(to_time_point_ns());
  }

  [[nodiscard]] TimePoint<Sec> to_time_point_sec() const {
    return std::chrono::time_point_cast<Sec>(to_time_point_ns());
  }

  [[nodiscard]] Ns to_duration_ns() const {
    return to_time_point_ns().time_since_epoch();
  }

  [[nodiscard]] Ms to_duration_ms() const {
    return std::chrono::duration_cast<Ms>(to_duration_ns());
  }

  [[nodiscard]] Sec to_duration_sec() const {
    return std::chrono::duration_cast<Sec>(to_duration_ns());
  }

  [[nodiscard]] int64_t to_ns() const { return value; }

  [[nodiscard]] double to_ms() const {
    return static_cast<double>(value) / 1000000.0;
  }

  [[nodiscard]] double to_sec() const {
    return static_cast<double>(value) / 1000000000.0;
  }

  Time operator+(Time const& rhs) const { return value + rhs.value; }

  Time operator-(Time const& rhs) const { return value - rhs.value; }

  bool operator>(Time const& rhs) const { return value > rhs.value; }

  bool operator<(Time const& rhs) const { return value < rhs.value; }

  bool operator>=(Time const& rhs) const { return value >= rhs.value; }

  bool operator<=(Time const& rhs) const { return value <= rhs.value; }

  bool operator==(Time const& rhs) const { return value == rhs.value; }

  bool operator!=(Time const& rhs) const { return !operator==(rhs.value); }

  static void set_now_function(std::function<Time()> func) {
    now_func_ = std::move(func);
  }

  static Time now() {
    if (now_func_.has_value()) {
      return now_func_.value()();
    }
    return Clock::now();
  }

  time_t value;

 private:
  static std::optional<std::function<Time()>> now_func_;
};

inline std::optional<std::function<Time()>> Time::now_func_ = {};

}  // namespace Slam