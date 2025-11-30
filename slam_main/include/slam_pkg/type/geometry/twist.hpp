//
// Created by park on 24. 4. 16.
//

#pragma once

#include "slam_pkg/type/statistics/covariance.hpp"
#include "vector.hpp"

namespace Slam {

struct Twist {
  Vector linear{};
  Vector angular{};
};

struct TwistWithCovariance {
  Twist twist{};
  Covariance<6> cov{};
};

}  // namespace Slam
