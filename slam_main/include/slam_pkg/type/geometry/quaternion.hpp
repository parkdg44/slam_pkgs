//
// Created by park on 24. 2. 14.
//

#pragma once

#include <eigen3/Eigen/Eigen>

#include "slam_pkg/type/geometry/point.hpp"

namespace Slam {

struct Quaternion {
  Quaternion() = default;

  Quaternion(double w, double x, double y, double z) : value(w, x, y, z) {}

  Quaternion(const Eigen::Quaterniond& q) : value(q) {}

  Quaternion(Eigen::Quaterniond&& q) : value(q) {}

  Quaternion(const Eigen::Matrix3d& q) : value(q) {}

  Quaternion(Eigen::Matrix3d&& q) : value(q) {}

  Quaternion(double r, double p, double y) : value(from_rpy(r, p, y).value) {}

  [[nodiscard]] Quaternion conjugate() const { return value.conjugate(); }

  [[nodiscard]] double norm() const { return value.norm(); }

  [[nodiscard]] Quaternion normalized() const { return value.normalized(); }

  [[nodiscard]] Quaternion inverse() const { return value.inverse(); }

  [[nodiscard]] Quaternion pow(double exponent) const {
    // reference:
    // https://en.wikipedia.org/wiki/Quaternion#Functions_of_a_quaternion_variable

    Eigen::AngleAxisd angle_axis{value};
    angle_axis.angle() *= exponent;
    return Eigen::Quaterniond{angle_axis};
  }

  [[nodiscard]] Point rotate(const Point& p) const {
    return {value.matrix() * p.value};
  }

  [[nodiscard]] Vector to_rpy() const { return {value.matrix().eulerAngles(0, 1, 2)}; }

  static Quaternion from_rpy(double r, double p, double y) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
    return {m};
  }

  Quaternion operator*(const Quaternion& q) const { return value * q.value; }

  Quaternion operator^(double value) const { return pow(value); }

  Eigen::Quaterniond value{};

  const double& x{value.x()};
  const double& y{value.y()};
  const double& z{value.z()};
  const double& w{value.w()};
};

}  // namespace Slam
