//
// Created by park on 24. 4. 16.
//

#pragma once

#include "vector.hpp"

namespace Slam {

struct Twist {
  Vector angular{};
  Vector linear{};
};

struct TwistWithCovariance {
  Twist twist{};
  Covariance<6> cov{};
};

}  // namespace Slam
