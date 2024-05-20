//
// Created by park on 24. 4. 16.
//

#pragma once

#include "pose.hpp"

namespace Slam {

struct Odometry {
  Header header;
  Pose pose;  
};

}  // namespace Slam