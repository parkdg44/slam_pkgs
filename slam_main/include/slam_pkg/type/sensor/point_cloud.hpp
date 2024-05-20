//
// Created by park on 24. 4. 17.
//

#pragma once

#include "slam_pkg/type/geometry/point.hpp"

namespace Slam {

template <typename T>
struct PointCloud {
  Header header{};
  std::vector<T> points{};
};

template struct PointCloud<Point>;

}  // namespace Slam