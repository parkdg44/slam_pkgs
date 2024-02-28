//
// Created by park on 24. 2. 14.
//

#pragma once

#include <cmath>

namespace Slam
{

  struct Position
  {
    double x{.0};
    double y{.0};
    double z{.0};

    [[nodiscard]] double hypot() const { return std::hypot(x, y, z); }
  };

} // namespace Slam
