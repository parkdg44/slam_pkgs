//
// Created by park on 24. 2. 14.
//

#pragma once

#include <eigen3/Eigen/Eigen>

namespace Slam {

class Point {
 public:
  template <class T>
  Point(T x, T y, T z) : value(x, y, z) {}

  Point(Eigen::Vector3d const& p) : value(p) {}

  Point(Eigen::Vector3d&& p) : value(p) {}

  Point operator-(Point const& rhs) const { return Point{value - rhs.value}; }

  Point operator+(Point const& rhs) const { return Point{value + rhs.value}; }

  [[nodiscard]] double hypot() const { return value.stableNorm(); }

  [[nodiscard]] double dist_to(Point const& p) const {
    return (p - *this).hypot();
  }

  Eigen::Vector3d value{};

  double& x{value.x()};
  double& y{value.y()};
  double& z{value.z()};
};

}  // namespace Slam
