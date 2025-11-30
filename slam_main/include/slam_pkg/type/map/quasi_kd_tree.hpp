#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "rdestl/hash_map.h"

namespace Slam {

static uint64_t part1by2(uint32_t n) {
  uint64_t x = n & 0x1fffff;
  x = (x | x << 32) & 0x1f00000000ffff;
  x = (x | x << 16) & 0x1f0000ff0000ff;
  x = (x | x << 8) & 0x100f00f00f00f00f;
  x = (x | x << 4) & 0x10c30c30c30c30c3;
  x = (x | x << 2) & 0x1249249249249249;
  return x;
}

uint64_t morton_encode(int x, int y, int z) {
  const uint32_t offset = 1U << 20;
  uint32_t ux = x + offset;
  uint32_t uy = y + offset;
  uint32_t uz = z + offset;

  return part1by2(ux) | (part1by2(uy) << 1) | (part1by2(uz) << 2);
}

template <typename, typename = std::void_t<>>
struct has_x_method : std::false_type {};

template <typename T>
struct has_x_method<T, std::void_t<decltype(std::declval<T>().x())>> : std::true_type {};

template <typename, typename = std::void_t<>>
struct has_y_method : std::false_type {};

template <typename T>
struct has_y_method<T, std::void_t<decltype(std::declval<T>().y())>> : std::true_type {};

template <typename, typename = std::void_t<>>
struct has_z_method : std::false_type {};

template <typename T>
struct has_z_method<T, std::void_t<decltype(std::declval<T>().z())>> : std::true_type {};

struct BenchMarkResult {
  std::chrono::duration<double, std::milli> voxel_lookup_time;
  std::chrono::duration<double, std::milli> candidate_search_time;
};

struct BuildBenchMarkResult {
  std::chrono::duration<double, std::milli> data_preparation_time;
  std::chrono::duration<double, std::milli> reverse_mapping_time;
  std::chrono::duration<double, std::milli> top_n_optimization_time;
};

template <typename T>
class QuasiKdTree {
  static_assert(has_x_method<T>::value, "Template type T must have a x() method.");
  static_assert(has_y_method<T>::value, "Template type T must have a y() method.");
  static_assert(has_z_method<T>::value, "Template type T must have a z() method.");

 public:
  struct NeighborInfo {
    std::shared_ptr<std::shared_mutex> mutex{std::make_shared<std::shared_mutex>()};
    std::vector<std::pair<double, std::weak_ptr<const T>>> neighbors;

    void insert(double dist, std::weak_ptr<const T> neighbor) {
      std::unique_lock lock(*mutex);
      neighbors.emplace_back(dist, neighbor);
    }
  };

  QuasiKdTree(double resolution, double max_distance, size_t top_n = 5)
      : resolution_(resolution),
        max_distance_(max_distance),
        top_n_(top_n),
        inv_resolution_(1.0 / resolution) {}

  void build(const std::vector<T>& raw_data, BuildBenchMarkResult* benchmark_result = nullptr) {
    if (raw_data.empty()) return;

    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. Prepare internal data storage
    raw_data_.clear();
    raw_data_.reserve(raw_data.size());
    for (const auto& pt : raw_data) {
      raw_data_.push_back(std::make_shared<const T>(pt));
    }

    auto after_prep = std::chrono::high_resolution_clock::now();

    // 2. Reverse mapping: Iterate through each point and register it to nearby voxels
    ptr_map_.clear();
    const int voxel_search_radius = static_cast<int>(std::ceil(max_distance_ * inv_resolution_));
    const double max_dist_sq = max_distance_ * max_distance_;

    for (size_t i = 0; i < raw_data_.size(); ++i) {
      const auto& point_ptr = raw_data_[i];
      const T& point = *point_ptr;
      int cx, cy, cz;
      getVoxelCoords(point, cx, cy, cz);

      for (int dz = -voxel_search_radius; dz <= voxel_search_radius; ++dz) {
        for (int dy = -voxel_search_radius; dy <= voxel_search_radius; ++dy) {
          for (int dx = -voxel_search_radius; dx <= voxel_search_radius; ++dx) {
            int nx = cx + dx;
            int ny = cy + dy;
            int nz = cz + dz;
            uint64_t morton_key = morton_encode(nx, ny, nz);
            PointCoord voxel_center = getVoxelCenter(nx, ny, nz);
            double dist_sq = distanceSq(point, voxel_center);

            if (dist_sq <= max_dist_sq) {
              ptr_map_[morton_key].neighbors.emplace_back(dist_sq, point_ptr);
            }
          }
        }
      }
    }

    auto after_mapping = std::chrono::high_resolution_clock::now();

    // 3. Top-N optimization: For each voxel, keep only the top_n nearest neighbors
    for (auto& pair : ptr_map_) {
      auto& info = pair.second;
      if (info.neighbors.size() > top_n_) {
        std::nth_element(info.neighbors.begin(), info.neighbors.begin() + top_n_,
                         info.neighbors.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });
        info.neighbors.erase(info.neighbors.begin() + top_n_, info.neighbors.end());
      }
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    if (benchmark_result) {
      benchmark_result->data_preparation_time = after_prep - start_time;
      benchmark_result->reverse_mapping_time = after_mapping - after_prep;
      benchmark_result->top_n_optimization_time = end_time - after_mapping;
    }
  }

  std::weak_ptr<const T> findNearest(const T& query,
                                     BenchMarkResult* benchmark_result = nullptr) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    int x, y, z;
    getVoxelCoords(query, x, y, z);
    uint64_t key = morton_encode(x, y, z);
    auto it = ptr_map_.find(key);

    auto after_lookup_time = std::chrono::high_resolution_clock::now();

    if (it == ptr_map_.end() || it->second.neighbors.empty()) {
      if (benchmark_result) {
        benchmark_result->voxel_lookup_time = after_lookup_time - start_time;
        benchmark_result->candidate_search_time = std::chrono::duration<double, std::milli>(0);
      }
      return {};
    }

    auto& info = it->second;

    const auto& candidates = info.neighbors;
    std::weak_ptr<const T> best_point;
    double min_dist_sq = -1.0;

    for (const auto& candidate_pair : candidates) {
      if (auto shared_candidate = candidate_pair.second.lock()) {
        double dist_sq = distanceSq(*shared_candidate, query);
        if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          best_point = candidate_pair.second;
        }
      }
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    if (benchmark_result) {
      benchmark_result->voxel_lookup_time = after_lookup_time - start_time;
      benchmark_result->candidate_search_time = end_time - after_lookup_time;
    }

    return best_point;
  }

 private:
  using coord_t = decltype(std::declval<T>().x());

  struct PointCoord {
    coord_t x, y, z;
  };

  double resolution_;
  double max_distance_;
  size_t top_n_;
  double inv_resolution_;

  std::vector<std::shared_ptr<const T>> raw_data_;
  mutable rde::hash_map<uint64_t, NeighborInfo> ptr_map_;

  void getVoxelCoords(const T& point, int& x, int& y, int& z) const {
    x = static_cast<int>(std::floor(point.x() * inv_resolution_));
    y = static_cast<int>(std::floor(point.y() * inv_resolution_));
    z = static_cast<int>(std::floor(point.z() * inv_resolution_));
  }

  PointCoord getVoxelCenter(int x, int y, int z) const {
    return {(static_cast<double>(x) + 0.5) * resolution_,
            (static_cast<double>(y) + 0.5) * resolution_,
            (static_cast<double>(z) + 0.5) * resolution_};
  }

  double distanceSq(const T& p1, const T& p2) const {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    double dz = p1.z() - p2.z();
    return dx * dx + dy * dy + dz * dz;
  }

  double distanceSq(const T& p1, const PointCoord& p2) const {
    double dx = p1.x() - p2.x;
    double dy = p1.y() - p2.y;
    double dz = p1.z() - p2.z;
    return dx * dx + dy * dy + dz * dz;
  }
};

}  // namespace Slam