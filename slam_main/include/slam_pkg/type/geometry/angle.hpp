//
// Created by park on 3/2/24.
//

#pragma once

#include "slam_pkg/util/math/math.hpp"
#include "vector.hpp"

namespace Slam {

struct Angle {
  Angle() = default;

  explicit Angle(double angle) : value(util::angle_constrain(angle)) {}

  template <typename T>
  bool operator==(T const& rhs) {
    return value == rhs;
  }

  Angle operator+(Angle const& rhs) const {
    return Angle(util::angle_constrain(value + rhs.value));
  }

  Angle operator-(Angle const& rhs) const {
    return Angle(util::angle_constrain(value - rhs.value));
  }

  Angle operator+() const { return Angle{+value}; }

  Angle operator-() const { return Angle{-value}; }

  double value{.0};
};

}  // namespace Slam
