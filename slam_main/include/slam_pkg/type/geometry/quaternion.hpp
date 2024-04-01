//
// Created by park on 24. 2. 14.
//

#pragma once

#include <eigen3/Eigen/Eigen>

namespace Slam {

class Quaternion {
 public:
  Quaternion(double x, double y, double z, double w) : value(x, y, z, w) {}

  Quaternion(Eigen::Quaterniond&& q) : value(q) {}

  Quaternion(const Eigen::Quaterniond& q) : value(q) {}

  [[nodiscard]] Eigen::Vector4d coeffs() const { return value.coeffs(); }

  [[nodiscard]] Quaternion conjugate() const { return value.conjugate(); }

  [[nodiscard]] double norm() const { return value.norm(); }

  [[nodiscard]] Quaternion normalized() const { return value.normalized(); }

  [[nodiscard]] Quaternion inverse() const { return value.inverse(); }

  [[nodiscard]] Quaternion pow(double exponent) const {
    Eigen::AngleAxisd angle_axis{value};
    angle_axis.angle() *= exponent;
    return Eigen::Quaterniond{angle_axis};
  }

  Quaternion operator*(const Quaternion& q) const { return value * q.value; }

  Quaternion operator^(double value) const { return this->pow(value); }

  Eigen::Quaterniond value;

  double& x{value.x()};
  double& y{value.y()};
  double& z{value.z()};
  double& w{value.w()};
};

}  // namespace Slam
