//
// Created by park on 24. 6. 13.
//

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace Slam {

struct Image {
  cv::Mat value;
};

}  // namespace Slam