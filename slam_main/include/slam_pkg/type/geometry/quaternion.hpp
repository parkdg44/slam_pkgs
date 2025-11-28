//
// Created by park on 24. 2. 14.
//

#pragma once

#include <Eigen/Geometry>

namespace Slam {

struct Quaternion {
  Eigen::Quaterniond value{Eigen::Quaterniond::Identity()};

  Quaternion() = default;

  Quaternion(double w, double x, double y, double z) : value(w, x, y, z) {}

  Quaternion(const Eigen::Quaterniond& q) : value(q) {}

  Quaternion(Eigen::Quaterniond&& q) : value(q) {}

  Quaternion(const Eigen::Matrix3d& R) : value(R) {}

  Quaternion(Eigen::Matrix3d&& R) : value(R) {}

  Quaternion(double r, double p, double y) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
    value = std::move(m);
  }

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

  [[nodiscard]] double angle_to(Quaternion q) const { return value.angularDistance(q.value); }

  static Quaternion lerp(const Quaternion& q0, const Quaternion& q1, double ratio) {
    // reference: Quaternion slerp, https://en.wikipedia.org/wiki/Slerp

    // q0_inv * q1
    Quaternion d = q0.inverse() * q1;
    // q0 * (q0_inv * q1)^t
    Quaternion q = q0 * (d ^ ratio);

    return q;
  }

  [[nodiscard]] Quaternion lerp(const Quaternion& q, double ratio) const {
    return lerp(*this, q, ratio);
  }

  Quaternion operator*(const Quaternion& q) const { return value * q.value; }

  Quaternion operator^(double value) const { return pow(value); }

  [[nodiscard]] double w() const { return value.w(); }
  [[nodiscard]] double x() const { return value.x(); }
  [[nodiscard]] double y() const { return value.y(); }
  [[nodiscard]] double z() const { return value.z(); }

  double& w() { return value.w(); }
  double& x() { return value.x(); }
  double& y() { return value.y(); }
  double& z() { return value.z(); }
};

}  // namespace Slam
