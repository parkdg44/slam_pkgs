//
// Created by park on 24. 4. 17.
//

#pragma once

#include "slam_pkg/type/geometry/header.hpp"
#include "slam_pkg/type/sensor/point_cloud.hpp"
#include "slam_pkg/type/types.hpp"

namespace Slam {
class Frame {
 public:
  Frame() = default;
  ~Frame() = default;

 public:
  PointCloud pcd_;
  Pose pose_;
  Header header_;
};
}  // namespace Slam