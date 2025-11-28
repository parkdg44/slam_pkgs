//
// Created by park on 24. 4. 17.
//

#pragma once

#include <vector>

#include "frame.hpp"

namespace Slam {
struct Graph {
  std::vector<Frame> frames;
};
}  // namespace Slam