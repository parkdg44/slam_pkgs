//
// Created by park on 24. 4. 17.
//

#pragma once

#include "slam_pkg/type/geometry/pose.hpp"
#include "slam_pkg/type/geometry/vector.hpp"
#include "slam_pkg/type/sensor/laser_scan.hpp"

namespace Slam {

struct PointCloud {
  Header header{};
  std::vector<Vector> points{};
};

}  // namespace Slam