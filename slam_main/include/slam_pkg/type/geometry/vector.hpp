//
// Created by park on 24. 2. 14.
//

#pragma once

#include <Eigen/Core>

namespace Slam {

struct Vector {
  Eigen::Vector3d value{};

  Vector() = default;

  Vector(double x, double y, double z) : value(x, y, z) {}

  Vector(Eigen::Vector3d const& v) : value(v) {}

  Vector(Eigen::Vector3d&& v) : value(v) {}

  /// functions

  [[nodiscard]] double norm() const { return value.stableNorm(); }

  [[nodiscard]] double dist_to(Vector const& p) const { return (p.value - value).stableNorm(); }

  [[nodiscard]] double dot(Vector const& p) const { return value.dot(p.value); }

  [[nodiscard]] Vector cross(Vector const& p) const { return value.cross(p.value); }

  static Vector lerp(const Vector& v0, const Vector& v1, double ratio) {
    // reference: lerp, https://en.wikipedia.org/wiki/Linear_interpolation
    return v0 + ratio * (v1 - v0);
  }

  Vector lerp(const Vector& v, double ratio) const { return lerp(*this, v, ratio); }

  /// operator overloads

  Vector operator+(Vector const& rhs) const { return {value + rhs.value}; }

  Vector operator-(Vector const& rhs) const { return {value - rhs.value}; }

  Vector operator*(double rhs) const { return {value * rhs}; }

  friend Vector operator*(double lhs, Vector const& v) { return {v.value * lhs}; }

  double operator*(Vector const& rhs) const { return dot(rhs); }

  Vector operator^(Vector const& rhs) const { return cross(rhs); }

  // accessors

  [[nodiscard]] double x() const { return value.x(); }
  [[nodiscard]] double y() const { return value.y(); }
  [[nodiscard]] double z() const { return value.z(); }

  double& x() { return value.x(); }
  double& y() { return value.y(); }
  double& z() { return value.z(); }
};

}  // namespace Slam
