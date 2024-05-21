//
// Created by park on 24. 4. 17.
//

#pragma once

#include "slam_pkg/type/geometry/point.hpp"
#include "slam_pkg/type/geometry/pose.hpp"

namespace Slam {

struct PointCloud {
  Header header{};
  std::vector<Point> points{};
};

}  // namespace Slam