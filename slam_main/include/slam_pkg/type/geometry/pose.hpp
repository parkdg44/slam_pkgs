//
// Created by park on 24. 4. 3.
//

#pragma once

#include "slam_pkg/type/geometry/point.hpp"
#include "slam_pkg/type/geometry/quaternion.hpp"
#include "slam_pkg/type/statistics/covariance.hpp"

namespace Slam {

struct Pose {
  Point p{};
  Quaternion q{};

  Pose operator+(Pose const& rhs) const { return {p + q.rotate(rhs.p), q * rhs.q}; }

  Point operator+(Point const& rhs) const { return {p + q.rotate(rhs)}; }

  [[nodiscard]] Pose inverse() const {
    // Use SE(3) inverse: R^T and -R^T t
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.value.toRotationMatrix();
    T.translation() = p.value;
    Eigen::Isometry3d Ti = T.inverse();
    return {Point{Ti.translation()}, Quaternion{Ti.linear()}};
  }

  Pose operator-(Pose const& rhs) const {
    // Relative transform: rhs^{-1} ∘ this
    Eigen::Isometry3d T_lhs = Eigen::Isometry3d::Identity();
    T_lhs.linear() = q.value.toRotationMatrix();
    T_lhs.translation() = p.value;

    Eigen::Isometry3d T_rhs = Eigen::Isometry3d::Identity();
    T_rhs.linear() = rhs.q.value.toRotationMatrix();
    T_rhs.translation() = rhs.p.value;

    Eigen::Isometry3d Trel = T_rhs.inverse() * T_lhs;
    return {Point{Trel.translation()}, Quaternion{Trel.linear()}};
  }
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
