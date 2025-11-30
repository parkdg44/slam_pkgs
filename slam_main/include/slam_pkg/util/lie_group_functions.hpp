/**
 * @file      lie_group_functions.hpp
 * @author    Donggyu Park (pksdd124@gmail.com)
 * @brief     Lie group related utility functions (Corrected)
 * @date      2025-11-28
 *
 * @copyright Copyright (c) 2025 Donggyu Park. All rights reserved.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace slam {

// Use inline to prevent ODR violations and code bloat in header-only files
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d v_hat;
  v_hat.row(0) << 0.0, v.z(), -v.y();
  v_hat.row(1) << -v.z(), 0.0, v.x();
  v_hat.row(2) << v.y(), -v.x(), 0.0;
  return v_hat;
}

// vee operator: use upper triangular part of the matrix
inline Eigen::Vector3d vee(const Eigen::Matrix3d& v_hat) {
  return Eigen::Vector3d(v_hat(2, 1), v_hat(0, 2), v_hat(1, 0));
}

namespace So3 {

inline Eigen::Quaterniond Exp(const Eigen::Vector3d& omega) {
  const double theta = omega.norm();
  // Use a slightly larger epsilon for numerical stability, usually 1e-7~1e-5 depending on precision
  // requirements
  if (theta < 1e-7) {
    // First order Taylor expansion could be used, but Identity is safe for very small angles
    return Eigen::Quaterniond::Identity();
  } else {
    return Eigen::Quaterniond(Eigen::AngleAxisd(theta, omega / theta));
  }
}

inline Eigen::Vector3d Log(const Eigen::Quaterniond& q) {
  Eigen::AngleAxisd angle_axis(q);
  return angle_axis.axis() * angle_axis.angle();
}

}  // namespace So3

namespace Se3 {

// Convention: xi = [rho, omega]^T (translation, rotation)
inline Eigen::Matrix4d Adjoint(const Eigen::Isometry3d& T) {
  Eigen::Matrix4d Adj = Eigen::Matrix4d::Zero();
  Eigen::Matrix3d R = T.rotation();
  Eigen::Vector3d t = T.translation();

  Adj.block<3, 3>(0, 0) = R;
  Adj.block<3, 3>(0, 3) = hat(t) * R;
  Adj.block<3, 3>(3, 3) = R;
  return Adj;
}

// Correct SE(3) Exp implementation with Jacobian V
inline Eigen::Isometry3d Exp(const Eigen::Matrix<double, 6, 1>& xi) {
  Eigen::Vector3d rho = xi.head<3>();    // Translation part of twist
  Eigen::Vector3d omega = xi.tail<3>();  // Rotation part of twist
  double theta = omega.norm();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R;
  Eigen::Matrix3d V;

  if (theta < 1e-7) {
    // Taylor expansion for small angles
    // R = I + omega^ + 0.5 * (omega^)^2
    // V = I + 0.5 * omega^
    Eigen::Matrix3d omega_hat = hat(omega);
    R = Eigen::Matrix3d::Identity() + omega_hat;
    V = Eigen::Matrix3d::Identity() + 0.5 * omega_hat;
  } else {
    Eigen::Matrix3d omega_hat = hat(omega);
    Eigen::Matrix3d omega_hat2 = omega_hat * omega_hat;
    double theta2 = theta * theta;
    double theta3 = theta2 * theta;

    // Rodrigues' formula
    R = Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * omega_hat +
        ((1.0 - std::cos(theta)) / theta2) * omega_hat2;

    // Left Jacobian V
    V = Eigen::Matrix3d::Identity() + ((1.0 - std::cos(theta)) / theta2) * omega_hat +
        ((theta - std::sin(theta)) / theta3) * omega_hat2;
  }

  T.linear() = R;
  T.translation() = V * rho;  // Apply V to rho
  return T;
}

// Correct SE(3) Log implementation with Inverse Jacobian V_inv
inline Eigen::Matrix<double, 6, 1> Log(const Eigen::Isometry3d& T) {
  Eigen::Matrix<double, 6, 1> xi;
  Eigen::Matrix3d R = T.rotation();
  Eigen::Vector3d t = T.translation();

  Eigen::AngleAxisd angle_axis(R);
  Eigen::Vector3d omega = angle_axis.axis() * angle_axis.angle();
  double theta = angle_axis.angle();

  Eigen::Matrix3d V_inv;

  if (theta < 1e-7) {
    Eigen::Matrix3d omega_hat = hat(omega);
    V_inv = Eigen::Matrix3d::Identity() - 0.5 * omega_hat;
  } else {
    Eigen::Matrix3d omega_hat = hat(omega);
    double theta2 = theta * theta;
    double half_theta = theta * 0.5;

    // V_inv = I - 0.5 * omega_hat + (1 - (theta * cos(theta/2)) / (2 * sin(theta/2))) ...
    // Simplified formula using cot:
    // V_inv = I - 0.5 * omega^ + (1/theta^2) * (1 - (theta/2) * cot(theta/2)) * (omega^)^2

    // double cot_term = std::tan(half_theta);  // cot(x) = 1/tan(x), check singularity? No,
    // theta!=0 Actually, simpler to write explicit coefficient:
    double scalar_term = (1.0 - half_theta * (1.0 / std::tan(half_theta))) / theta2;

    V_inv = Eigen::Matrix3d::Identity() - 0.5 * omega_hat + scalar_term * (omega_hat * omega_hat);
  }

  xi.head<3>() = V_inv * t;
  xi.tail<3>() = omega;

  return xi;
}

}  // namespace Se3

}  // namespace slam