//
// Created by park on 24. 2. 14.
//

#pragma once

#include <eigen3/Eigen/Eigen>

#include "util/math.hpp"

namespace Slam {

class Vector {
 public:
  Vector(double x, double y, double z) : value(x, y, z) {}

  Vector(Eigen::Vector3d const& v) : value(v) {}

  Vector(Eigen::Vector3d&& v) : value(v) {}

  [[nodiscard]] double norm() const { return value.stableNorm(); }

  [[nodiscard]] double dist_to(Vector const& p) const {
    return (p.value - value).stableNorm();
  }

  [[nodiscard]] double dot(Vector const& p) const { return value.dot(p.value); }

  [[nodiscard]] Vector cross(Vector const& p) const {
    return value.cross(p.value);
  }

  [[nodiscard]] double angle_to(Vector const& p) const {
    return math::angle_constrain(acos(dot(p) / (norm() * p.norm())));
  }

  Vector operator+(Vector const& rhs) const { return {value + rhs.value}; }

  Vector operator-(Vector const& rhs) const { return {value - rhs.value}; }

  template <typename T>
  Vector operator*(T rhs) const {
    return {value * rhs};
  }

  Eigen::Vector3d value{};

  double& x{value.x()};
  double& y{value.y()};
  double& z{value.z()};
};

// wrapper for scalar lhs value.
template <typename T>
Vector operator*(T lhs, Vector const& v) {
  return v * lhs;
}

}  // namespace Slam
