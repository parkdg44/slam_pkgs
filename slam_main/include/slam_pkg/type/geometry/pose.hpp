//
// Created by park on 24. 4. 3.
//

#pragma once

#include "type/geometry/point.hpp"
#include "type/geometry/quaternion.hpp"
#include "type/time/time.hpp"

namespace Slam {

struct Pose {
  Header header{};
  Point p{};
  Quaternion q{};
};

}  // namespace Slam
