//
// Created by park on 24. 6. 25.
//

#pragma once

#include <Eigen/Eigen>

#include "slam_pkg/type/geometry/header.hpp"

namespace Slam {

struct CameraInfo {
  Header header;
  uint height;
  uint width;
  std::string distortion_model;
  std::vector<double> d;
  Eigen::Matrix3d k;
  Eigen::Matrix3d r;
  Eigen::Matrix<double, 3, 4> p;
};

}  // namespace Slam