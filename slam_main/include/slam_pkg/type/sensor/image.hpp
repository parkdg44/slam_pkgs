//
// Created by park on 24. 6. 13.
//

#pragma once

#include <opencv2/opencv.hpp>

#include "slam_pkg/type/geometry/header.hpp"

namespace Slam {

struct Image {
  Header header;
  std::string encoding;
  cv::Mat value;
};

}  // namespace Slam