//
// Created by park on 24. 4. 16.
//

#pragma once

#include "header.hpp"
#include "pose.hpp"
#include "twist.hpp"

namespace Slam {

struct Odometry {
  Header header{};
  PoseWithCovariance pose_w_cov{};
  TwistWithCovariance twist_w_cov{};
};

}  // namespace Slam