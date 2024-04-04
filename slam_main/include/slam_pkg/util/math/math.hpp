//
// Created by park on 24. 4. 1.
//

#pragma once

#include "cmath"

namespace Slam::math {

inline double angle_constrain(double x) {
  static constexpr double pi = M_PI;
  static constexpr double pi_double = 2 * pi;

  x = fmod(x + pi, pi_double);
  if (x < 0) x += pi_double;
  return x - pi;
}

inline double angle_diff(double a, double b) { return angle_constrain(b - a); }

}  // namespace Slam::math