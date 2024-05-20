//
// Created by park on 24. 4. 3.
//

#pragma once

#include "slam_pkg/type/geometry/point.hpp"
#include "slam_pkg/type/geometry/quaternion.hpp"

namespace Slam {

struct Pose {
  Point p{};
  Quaternion q{};

  Pose operator*(Pose const& rhs) const { return {p + q.rotate(rhs.p), q * rhs.q}; }
};

struct PoseStamped {
  Header header;
  Pose pose;
};

}  // namespace Slam
