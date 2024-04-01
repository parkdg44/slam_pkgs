//
// Created by park on 2/28/24.
//

#pragma once

#include "type/types.hpp"
#include "util/math.hpp"

namespace Slam::math {

inline Quaternion slerp(const Quaternion& q0, const Quaternion& q1,
                        double ratio) {
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

}  // namespace Slam::math
