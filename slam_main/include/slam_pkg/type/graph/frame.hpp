//
// Created by park on 24. 4. 17.
//

#pragma once

#include <nanoflann/nanoflann.hpp>

#include "slam_pkg/type/geometry/header.hpp"
#include "slam_pkg/type/sensor/point_cloud.hpp"

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