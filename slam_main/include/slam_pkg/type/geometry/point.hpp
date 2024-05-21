//
// Created by park on 24. 2. 14.
//

#pragma once

#include <eigen3/Eigen/Eigen>

namespace Slam {

struct Point {
  Point() = default;

  Point(double x, double y, double z) : value(x, y, z) {}

  Point(Eigen::Vector3d const& p) : value(p) {}

  Point(Eigen::Vector3d&& p) noexcept : value(p) {}

  Point operator-() const { return Point{-value}; }

  Point operator-(Point const& rhs) const { return Point{value - rhs.value}; }

  Point operator+(Point const& rhs) const { return Point{value + rhs.value}; }

  [[nodiscard]] double norm() const { return value.stableNorm(); }

  [[nodiscard]] double dist_to(Point const& p) const { return (p - *this).norm(); }

  Point& operator=(const Point& p) {
    value = p.value;
    return *this;
  }

  Eigen::Vector3d value{};

  double& x{value.x()};
  double& y{value.y()};
  double& z{value.z()};
};

}  // namespace Slam
