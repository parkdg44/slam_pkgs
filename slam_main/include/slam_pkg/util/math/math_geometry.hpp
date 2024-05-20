//
// Created by park on 2/28/24.
//

#pragma once

#include "slam_pkg/type/types.hpp"

namespace Slam::util {

inline Quaternion to_quat(const Vector& v1, const Vector& v2) {
  return Eigen::Quaterniond::FromTwoVectors(v1.value, v2.value);
}

inline Quaternion to_quat(const Vector& v1) {
  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{1.0, 0, 0}, v1.value);
}

inline Vector to_vector(const Quaternion& q) { return q.rotate({1, 0, 0}).value; }

inline Quaternion slerp(const Quaternion& q0, const Quaternion& q1, double ratio) {
  // reference: Quaternion slerp, https://en.wikipedia.org/wiki/Slerp

  // q0_inv * q1
  Quaternion d = q0.inverse() * q1;
  // q0 * (q0_inv * q1)^t
  Quaternion q = q0 * (d ^ ratio);

  return q;
}

inline Vector lerp(const Vector& p0, const Vector& p1, double ratio) {
  // reference: lerp, https://en.wikipedia.org/wiki/Linear_interpolation
  return p0 + ratio * (p1 - p0);
}

}  // namespace Slam::util
