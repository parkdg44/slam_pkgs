//
// Created by park on 24. 4. 3.
//

#pragma once

#include "slam_pkg/type/geometry/header.hpp"
#include "slam_pkg/type/geometry/quaternion.hpp"
#include "slam_pkg/type/geometry/twist.hpp"
#include "slam_pkg/type/geometry/vector.hpp"
#include "slam_pkg/type/statistics/covariance.hpp"

namespace Slam {

struct Pose {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();

  Pose() = default;

  Pose(Eigen::Isometry3d const& iso) : value(iso) {}

  Pose(Eigen::Vector3d const& p, Eigen::Quaterniond const& q) {
    value.linear() = q.toRotationMatrix();
    value.translation() = p;
  }

  Pose(Vector const& p, Quaternion const& q) {
    value.linear() = q.value.toRotationMatrix();
    value.translation() = p.value;
  }

  Vector translation() const { return Vector{value.translation()}; }

  Quaternion rotation() const { return Quaternion{value.rotation()}; }

  Twist se3() const {
    Eigen::AngleAxisd angle_axis{value.rotation()};
    Eigen::Vector3d omega = angle_axis.axis() * angle_axis.angle();
    return Twist{Vector{value.translation()}, Vector{omega}};
  }

  [[nodiscard]] Pose inverse() const { return Pose{value.inverse()}; }

  Pose operator+(Pose const& rhs) const { return Pose{this->value * rhs.value}; }

  Vector operator+(Vector const& rhs) const { return Vector{this->value * rhs.value}; }

  Pose operator-() const { return this->inverse(); }

  // Relative transform: rhs^{-1} ∘ this
  Pose operator-(Pose const& rhs) const { return Pose{rhs.value.inverse() * this->value}; }
};

struct PoseStamped {
  Header header{};
  Pose pose{};
};

struct PoseWithCovariance {
  Pose pose{};
  Covariance<6> cov{};
};

}  // namespace Slam
