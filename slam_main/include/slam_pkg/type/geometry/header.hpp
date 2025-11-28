//
// Created by park on 24. 4. 3.
//

#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <string>

namespace Slam {

struct Time {
  Time() = default;

  Time(int64_t t) : value(t) {}

  [[nodiscard]] int64_t to_ns() const { return value; }

  [[nodiscard]] double to_ms() const { return static_cast<double>(value) / 1000000.0; }

  [[nodiscard]] double to_sec() const { return static_cast<double>(value) / 1000000000.0; }

  Time operator+(Time const& rhs) const { return value + rhs.value; }

  Time operator-(Time const& rhs) const { return value - rhs.value; }

  bool operator>(Time const& rhs) const { return value > rhs.value; }

  bool operator<(Time const& rhs) const { return value < rhs.value; }

  bool operator>=(Time const& rhs) const { return value >= rhs.value; }

  bool operator<=(Time const& rhs) const { return value <= rhs.value; }

  bool operator==(Time const& rhs) const { return value == rhs.value; }

  bool operator!=(Time const& rhs) const { return !operator==(rhs.value); }

  static void set_now_function(std::function<Time()> func) { _now_func_ = std::move(func); }

  static Time system_now() { return std::chrono::system_clock::now().time_since_epoch().count(); }

  static Time now() {
    if (_now_func_.has_value()) {
      return _now_func_.value()();
    }
    return system_now();
  }

  int64_t value;

 private:
  static std::optional<std::function<Time()>> _now_func_;
};

inline std::optional<std::function<Time()>> Time::_now_func_ = {};

struct Header {
  Time stamp{};
  std::string ref_frame_id{};
};

}  // namespace Slam