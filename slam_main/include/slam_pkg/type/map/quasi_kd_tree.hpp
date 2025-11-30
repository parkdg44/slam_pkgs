#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <execution>
#include <iostream>
#include <limits>
#include <memory>
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
  std::chrono::duration<double, std::milli> merging_time;
  std::chrono::duration<double, std::milli> linear_build_time;
};

template <typename T>
class QuasiKdTree {
  static_assert(has_x_method<T>::value, "Template type T must have a x() method.");
  static_assert(has_y_method<T>::value, "Template type T must have a y() method.");
  static_assert(has_z_method<T>::value, "Template type T must have a z() method.");

 public:
  struct VoxelRange {
    uint32_t start_index;
    uint32_t count;
  };

  struct Entry {
    uint64_t key;
    float dist_sq;
    uint32_t index;
    float x, y, z;

    bool operator<(const Entry& other) const {
      if (key != other.key) return key < other.key;
      return dist_sq < other.dist_sq;
    }
  };

  struct Shard {
    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    std::vector<uint32_t> indices;
    rde::hash_map<uint64_t, VoxelRange> map;
  };

  QuasiKdTree(double resolution, double max_distance, size_t top_n = 3)
      : resolution_(resolution),
        max_distance_(max_distance),
        top_n_(top_n),
        inv_resolution_(1.0 / resolution) {}

  void build(const std::vector<T>& raw_data, BuildBenchMarkResult* benchmark_result = nullptr) {
    if (raw_data.empty()) return;

    auto start_time = std::chrono::high_resolution_clock::now();

    raw_data_ = raw_data;

    auto after_prep = std::chrono::high_resolution_clock::now();

    unsigned int n_threads = std::thread::hardware_concurrency();
    if (n_threads == 0) n_threads = 4;

    std::vector<std::vector<Entry>> thread_buffers(n_threads);

    const int voxel_search_radius = static_cast<int>(std::ceil(max_distance_ * inv_resolution_));
    const double max_dist_sq = max_distance_ * max_distance_;

    auto process_chunk = [&](size_t start, size_t end, unsigned int thread_id) {
      auto& buffer = thread_buffers[thread_id];
      buffer.reserve((end - start) * 27);

      for (size_t i = start; i < end; ++i) {
        const T& point = raw_data_[i];

        float px = static_cast<float>(point.x());
        float py = static_cast<float>(point.y());
        float pz = static_cast<float>(point.z());

        int cx, cy, cz;
        getVoxelCoords(point, cx, cy, cz);

        for (int dz = -voxel_search_radius; dz <= voxel_search_radius; ++dz) {
          for (int dy = -voxel_search_radius; dy <= voxel_search_radius; ++dy) {
            for (int dx = -voxel_search_radius; dx <= voxel_search_radius; ++dx) {
              int nx = cx + dx;
              int ny = cy + dy;
              int nz = cz + dz;

              PointCoord voxel_center = getVoxelCenter(nx, ny, nz);
              double dist_sq = distanceSq(point, voxel_center);

              if (dist_sq <= max_dist_sq) {
                buffer.push_back({morton_encode(nx, ny, nz), static_cast<float>(dist_sq),
                                  static_cast<uint32_t>(i), px, py, pz});
              }
            }
          }
        }
      }
    };

    std::vector<std::thread> threads;
    size_t chunk_size = raw_data_.size() / n_threads;
    for (unsigned int i = 0; i < n_threads; ++i) {
      size_t start = i * chunk_size;
      size_t end = (i == n_threads - 1) ? raw_data_.size() : (i + 1) * chunk_size;
      threads.emplace_back(process_chunk, start, end, i);
    }
    for (auto& t : threads) t.join();

    auto after_mapping = std::chrono::high_resolution_clock::now();

    size_t total_size = 0;
    for (const auto& buf : thread_buffers) total_size += buf.size();

    std::vector<Entry> all_entries;
    all_entries.reserve(total_size);

    for (const auto& buf : thread_buffers) {
      all_entries.insert(all_entries.end(), buf.begin(), buf.end());
    }

    std::sort(std::execution::par_unseq, all_entries.begin(), all_entries.end());

    auto after_merging = std::chrono::high_resolution_clock::now();

    shards_.resize(n_threads);
    shard_max_keys_.resize(n_threads);

    std::vector<size_t> split_indices(n_threads + 1, 0);
    split_indices[n_threads] = all_entries.size();

    size_t ideal_shard_size = all_entries.size() / n_threads;
    for (unsigned int i = 1; i < n_threads; ++i) {
      size_t idx = i * ideal_shard_size;
      while (idx < all_entries.size() && idx > 0 &&
             all_entries[idx].key == all_entries[idx - 1].key) {
        idx++;
      }
      split_indices[i] = idx;
    }

    auto build_shard = [&](unsigned int thread_id) {
      size_t start = split_indices[thread_id];
      size_t end = split_indices[thread_id + 1];
      if (start >= end) return;

      auto& shard = shards_[thread_id];
      shard.map.clear();

      size_t estimated_size = (end - start) / 2;
      shard.xs.reserve(estimated_size);
      shard.ys.reserve(estimated_size);
      shard.zs.reserve(estimated_size);
      shard.indices.reserve(estimated_size);

      uint64_t current_key = all_entries[start].key;
      size_t current_count = 0;
      size_t start_idx = 0;

      auto add_point = [&](const Entry& e) {
        shard.xs.push_back(e.x);
        shard.ys.push_back(e.y);
        shard.zs.push_back(e.z);
        shard.indices.push_back(e.index);
      };

      add_point(all_entries[start]);
      current_count++;

      for (size_t i = start + 1; i < end; ++i) {
        const auto& entry = all_entries[i];
        if (entry.key == current_key) {
          if (current_count < top_n_) {
            add_point(entry);
            current_count++;
          }
        } else {
          shard.map[current_key] = {static_cast<uint32_t>(start_idx),
                                    static_cast<uint32_t>(current_count)};

          current_key = entry.key;
          current_count = 1;
          start_idx = shard.xs.size();
          add_point(entry);
        }
      }

      shard.map[current_key] = {static_cast<uint32_t>(start_idx),
                                static_cast<uint32_t>(current_count)};
      shard_max_keys_[thread_id] = current_key;
    };

    std::vector<std::thread> build_threads;
    for (unsigned int i = 0; i < n_threads; ++i) {
      build_threads.emplace_back(build_shard, i);
    }
    for (auto& t : build_threads) t.join();

    auto end_time = std::chrono::high_resolution_clock::now();

    if (benchmark_result) {
      benchmark_result->data_preparation_time = after_prep - start_time;
      benchmark_result->reverse_mapping_time = after_mapping - after_prep;
      benchmark_result->merging_time = after_merging - after_mapping;
      benchmark_result->linear_build_time = end_time - after_merging;
    }
  }

  const T* findNearest(const T& query, BenchMarkResult* benchmark_result = nullptr) const {
    auto start_time = std::chrono::high_resolution_clock::now();

    int x, y, z;
    getVoxelCoords(query, x, y, z);
    uint64_t key = morton_encode(x, y, z);

    auto shard_it = std::lower_bound(shard_max_keys_.begin(), shard_max_keys_.end(), key);

    const T* best_point = nullptr;
    auto after_lookup_time = std::chrono::high_resolution_clock::now();

    if (shard_it != shard_max_keys_.end()) {
      size_t shard_idx = std::distance(shard_max_keys_.begin(), shard_it);
      const auto& shard = shards_[shard_idx];

      auto it = shard.map.find(key);
      if (it != shard.map.end() && it->second.count > 0) {
        uint32_t start_idx = it->second.start_index;
        uint32_t count = it->second.count;

        float qx = static_cast<float>(query.x());
        float qy = static_cast<float>(query.y());
        float qz = static_cast<float>(query.z());

        float min_dist_sq = std::numeric_limits<float>::max();
        uint32_t best_local_idx = static_cast<uint32_t>(-1);

        const float* ptr_x = &shard.xs[start_idx];
        const float* ptr_y = &shard.ys[start_idx];
        const float* ptr_z = &shard.zs[start_idx];

        for (uint32_t i = 0; i < count; ++i) {
          float dx = ptr_x[i] - qx;
          float dy = ptr_y[i] - qy;
          float dz = ptr_z[i] - qz;
          float dist_sq = dx * dx + dy * dy + dz * dz;

          if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_local_idx = i;
          }
        }

        if (best_local_idx != static_cast<uint32_t>(-1)) {
          uint32_t real_idx = shard.indices[start_idx + best_local_idx];
          best_point = &raw_data_[real_idx];
        }
      }
    }

    auto after_candidate_search = std::chrono::high_resolution_clock::now();

    if (benchmark_result) {
      benchmark_result->voxel_lookup_time = after_lookup_time - start_time;
      benchmark_result->candidate_search_time = after_candidate_search - after_lookup_time;
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

  std::vector<T> raw_data_;
  std::vector<Shard> shards_;
  std::vector<uint64_t> shard_max_keys_;

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