//
// Created by park on 24. 4. 17.
//

#pragma once

#include "slam_pkg/type/geometry/point.hpp"
#include "slam_pkg/type/geometry/pose.hpp"
#include "slam_pkg/type/sensor/laser_scan.hpp"

namespace Slam {

struct MatrixInfo {
  int width{-1};
  int height{-1};

  [[nodiscard]] uint idx(uint row, uint col) const { return row * width + col; }
};

struct PointCloud {
  Header header{};
  std::vector<Point> points{};
  MatrixInfo info{};
};

template <typename T>
struct ScanPointCloud {
  LaserScan scan{};
  std::vector<Point> points{};
  const Header& header{scan.header};
};

}  // namespace Slam