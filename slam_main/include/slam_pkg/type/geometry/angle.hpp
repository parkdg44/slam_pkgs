//
// Created by park on 3/2/24.
//

#pragma once

#include "util/math/math.hpp"
#include "vector.hpp"

namespace Slam {

struct Angle {
  Angle() = default;

  Angle(double angle) : value(math::angle_constrain(angle)) {}

  Angle(Vector const& v1, Vector const& v2) : value(v1.angle_to(v2)) {}

  template <typename T>
  bool operator==(T const& rhs) {
    return value == rhs;
  }

  Angle operator+(Angle const& rhs) const {
    return math::angle_constrain(value + rhs.value);
  }

  Angle operator-(Angle const& rhs) const {
    return math::angle_constrain(value - rhs.value);
  }

  Angle operator+() const { return Angle{+value}; }

  Angle operator-() const { return Angle{-value}; }

  double value{.0};
};

}  // namespace Slam
