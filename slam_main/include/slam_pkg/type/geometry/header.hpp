//
// Created by park on 24. 4. 3.
//

#pragma once

#include <string>

#include "slam_pkg/type/time/time.hpp"

namespace Slam {

struct Header {
  Time stamp{};
  std::string ref_frame_id{};
};

}  // namespace Slam