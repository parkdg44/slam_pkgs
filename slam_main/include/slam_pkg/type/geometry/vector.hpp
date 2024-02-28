//
// Created by park on 24. 2. 14.
//

#pragma once

#include <cmath>

namespace Slam
{
  struct Vector
  {
    double x{.0};
    double y{.0};
    double z{.0};

    [[nodiscard]] double hypot() const { return std::hypot(x, y, z); }

    [[nodiscard]] double dot(Vector const &v) const { return {v.x * x + v.y * y + v.z * z}; }

    [[nodiscard]] Vector cross(Vector const &v) const
    {
      // reference: https://en.wikipedia.org/wiki/Cross_product#Computing
      //            (a2*b3 - a3*b2)i + (a3*b1 - a1*b3)j + (a1*b2 - a2*b1)z
      return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }

    double operator*(Vector const &v) const { return dot(v); }

    Vector operator^(Vector const &v) const { return cross(v); }
  };
} // namespace Slam
