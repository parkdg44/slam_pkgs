//
// Created by park on 24. 4. 17.
//

#pragma once

#include <vector>

#include "slam_pkg/type/geometry/header.hpp"

namespace Slam {

struct ScanInfo {
  bool is_ccw{true};
  double angle_min{.0}, angle_max{.0};
  double angle_increment{.0};
  double time_increment{.0};
  double scan_time{.0};
  double range_min{.0}, range_max{.0};
};

struct LaserScan {
  ScanInfo info;
  Header header;
  std::vector<float> ranges{};
  std::vector<float> intensities{};
};

}  // namespace Slam