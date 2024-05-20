//
// Created by park on 24. 2. 14.
//

#pragma once

#include <eigen3/Eigen/Eigen>

#include "slam_pkg/util/math/math.hpp"

namespace Slam {

struct Vector {
  Vector() = default;

  Vector(double x, double y, double z) : value(x, y, z) {}

  Vector(Eigen::Vector3d const& v) : value(v) {}

  Vector(Eigen::Vector3d&& v) : value(v) {}

  [[nodiscard]] double norm() const { return value.stableNorm(); }

  [[nodiscard]] double dist_to(Vector const& p) const { return (p.value - value).stableNorm(); }

  [[nodiscard]] double dot(Vector const& p) const { return value.dot(p.value); }

  [[nodiscard]] Vector cross(Vector const& p) const { return value.cross(p.value); }

  [[nodiscard]] double angle_to(Vector const& p) const {
    return util::angle_constrain(acos(dot(p) / (norm() * p.norm())));
  }

  Vector operator+(Vector const& rhs) const { return {value + rhs.value}; }

  Vector operator-(Vector const& rhs) const { return {value - rhs.value}; }

  double operator*(Vector const& rhs) const { return dot(rhs); }

  Vector operator^(Vector const& rhs) const { return cross(rhs); }

  Eigen::Vector3d value{};

  const double& x{value.x()};
  const double& y{value.y()};
  const double& z{value.z()};
};

// wrapper for scalar lhs value.
inline Vector operator*(double lhs, Vector const& v) { return {v.value * lhs}; }

}  // namespace Slam
