#include <chrono>
#include <future>
#include <gtest/gtest.h>
#include <iomanip>  // For std::fixed and std::setprecision
#include <random>
#include <sstream>
#include <thread>
#include <vector>

#include "slam_pkg/type/map/quasi_kd_tree.hpp"
#include "utils/kdtree.h"

// A simple point struct for benchmarking that satisfies the QuasiKdTree
// requirements.
struct BenchmarkPoint {
 private:
  double x_ = 0, y_ = 0, z_ = 0;

 public:
  BenchmarkPoint(double x = 0, double y = 0, double z = 0) : x_(x), y_(y), z_(z) {}

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
};

// Helper function to generate random points
void generate_random_points(std::vector<BenchmarkPoint>& points, size_t count, double max_coord) {
  points.resize(count);
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(-max_coord, max_coord);

  for (size_t i = 0; i < count; ++i) {
    points[i] = BenchmarkPoint(dis(gen), dis(gen), dis(gen));
  }
}

class BenchmarkTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 1,000,000 points
    generate_random_points(map_points_, 1000000, 100.0);
    generate_random_points(query_points_, 1000, 100.0);
  }

  std::vector<BenchmarkPoint> map_points_;
  std::vector<BenchmarkPoint> query_points_;
};

TEST_F(BenchmarkTest, Comparison) {
  const std::chrono::seconds timeout(300);
  auto test_future = std::async(std::launch::async, [this]() -> std::string {
    // --- KDTree Setup (Ground Truth) ---
    PointCloud<double> cloud;
    cloud.pts.resize(map_points_.size());
    for (size_t i = 0; i < map_points_.size(); ++i) {
      cloud.pts[i] = {map_points_[i].x(), map_points_[i].y(), map_points_[i].z()};
    }

    auto start_build_kdtree = std::chrono::high_resolution_clock::now();
    KDTree<double> kd_tree(cloud);
    auto end_build_kdtree = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> build_duration_kdtree =
        end_build_kdtree - start_build_kdtree;
    std::cout << "[KDTree] Build time: " << build_duration_kdtree.count() << " ms" << std::endl;

    // --- QuasiKdTree Setup ---
    Slam::BuildBenchMarkResult build_benchmark;
    auto start_build_quasi = std::chrono::high_resolution_clock::now();
    double max_distance = 0.5;
    Slam::QuasiKdTree<BenchmarkPoint> quasi_kd_tree(
        0.25, max_distance,
        5);  // resolution=1.0, max_distance=3.0, top_n=5
    quasi_kd_tree.build(map_points_, &build_benchmark);
    auto end_build_quasi = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> build_duration_quasi =
        end_build_quasi - start_build_quasi;
    std::cout << "[QuasiKdTree] Build time: " << build_duration_quasi.count() << " ms" << std::endl;
    std::cout << "    - Data Preparation: " << build_benchmark.data_preparation_time.count()
              << " ms" << std::endl;
    std::cout << "    - Reverse Mapping: " << build_benchmark.reverse_mapping_time.count() << " ms"
              << std::endl;
    std::cout << "    - Top-N Optimization: " << build_benchmark.top_n_optimization_time.count()
              << " ms" << std::endl;

    // --- Query and Compare ---
    std::vector<std::pair<const PointCloud<double>::Point*, double>> kdtree_results;
    kdtree_results.reserve(query_points_.size());

    auto start_query_kdtree = std::chrono::high_resolution_clock::now();
    for (const auto& q_point : query_points_) {
      const double query_pt_arr[3] = {q_point.x(), q_point.y(), q_point.z()};
      auto nearest_pair = kd_tree.findNearest(query_pt_arr);
      kdtree_results.emplace_back(&cloud.pts[nearest_pair.first], std::sqrt(nearest_pair.second));
    }
    auto end_query_kdtree = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> query_duration_kdtree =
        end_query_kdtree - start_query_kdtree;
    std::cout << "[KDTree] Query time for " << query_points_.size()
              << " points: " << query_duration_kdtree.count() << " ms" << std::endl;

    int correct_matches = 0;
    int failed_queries = 0;
    int incorrect_matches = 0;
    int correct_nulls = 0;
    double total_error_dist = 0.0;

    std::chrono::duration<double, std::milli> find_nearest_duration_total(0);
    std::chrono::duration<double, std::milli> comparison_duration_total(0);
    Slam::BenchMarkResult find_nearest_benchmark_total;
    find_nearest_benchmark_total.voxel_lookup_time = std::chrono::duration<double, std::milli>(0);
    find_nearest_benchmark_total.candidate_search_time =
        std::chrono::duration<double, std::milli>(0);

    auto start_query_quasi = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < query_points_.size(); ++i) {
      const auto& q_point = query_points_[i];

      auto find_nearest_start = std::chrono::high_resolution_clock::now();
      Slam::BenchMarkResult find_nearest_benchmark_single;
      auto nearest_ptr = quasi_kd_tree.findNearest(q_point, &find_nearest_benchmark_single).lock();
      auto find_nearest_end = std::chrono::high_resolution_clock::now();
      find_nearest_duration_total += (find_nearest_end - find_nearest_start);
      find_nearest_benchmark_total.voxel_lookup_time +=
          find_nearest_benchmark_single.voxel_lookup_time;
      find_nearest_benchmark_total.candidate_search_time +=
          find_nearest_benchmark_single.candidate_search_time;

      auto comparison_start = std::chrono::high_resolution_clock::now();
      const auto* kdtree_point = kdtree_results[i].first;

      if (nearest_ptr == nullptr) {
        // It's a failure only if a point should have been found within the
        // max_distance
        if (kdtree_results[i].second < max_distance) {
          failed_queries++;
        } else {
          correct_nulls++;
        }
        // Otherwise, it correctly returned null for a point in an empty region.
      } else {
        // A point was found, check if it's the correct one.
        if (std::abs(kdtree_point->x - nearest_ptr->x()) < 1e-9 &&
            std::abs(kdtree_point->y - nearest_ptr->y()) < 1e-9 &&
            std::abs(kdtree_point->z - nearest_ptr->z()) < 1e-9) {
          correct_matches++;
        } else {
          incorrect_matches++;
          double kdtree_dist = kdtree_results[i].second;
          double dx = nearest_ptr->x() - q_point.x();
          double dy = nearest_ptr->y() - q_point.y();
          double dz = nearest_ptr->z() - q_point.z();
          double quasi_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
          total_error_dist += (quasi_dist - kdtree_dist);
        }
      }
      auto comparison_end = std::chrono::high_resolution_clock::now();
      comparison_duration_total += (comparison_end - comparison_start);
    }
    auto end_query_quasi = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> query_duration_quasi =
        end_query_quasi - start_query_quasi;
    std::cout << "[QuasiKdTree] Query time for " << query_points_.size()
              << " points: " << query_duration_quasi.count() << " ms" << std::endl;
    std::cout << "[QuasiKdTree]   - findNearest total: " << find_nearest_duration_total.count()
              << " ms" << std::endl;
    std::cout << "    - Voxel Lookup total: "
              << find_nearest_benchmark_total.voxel_lookup_time.count() << " ms" << std::endl;
    std::cout << "    - Candidate Search total: "
              << find_nearest_benchmark_total.candidate_search_time.count() << " ms" << std::endl;
    std::cout << "[QuasiKdTree]   - comparison total: " << comparison_duration_total.count()
              << " ms" << std::endl;

    // --- Report Accuracy ---
    double avg_error = (incorrect_matches > 0) ? (total_error_dist / incorrect_matches) : 0.0;
    int total_correct = correct_matches + correct_nulls;

    std::cout << "\n--- Accuracy Report ---\n"
              << "Total Queries: " << query_points_.size() << "\n"
              << std::fixed << std::setprecision(2) << "Correct Matches: " << correct_matches
              << "\n"
              << "Correct Nulls: " << correct_nulls << "\n"
              << "Accuracy (Correct Matches + Correct Nulls): " << total_correct << " ("
              << (100.0 * total_correct / query_points_.size()) << "%)\n"
              << "Incorrect Matches: " << incorrect_matches << "\n"
              << "Failed Queries (null): " << failed_queries << "\n"
              << "Average Distance Error (for incorrect matches): " << avg_error << "\n"
              << "-----------------------\n";

    return "";  // Success
  });

  if (test_future.wait_for(timeout) == std::future_status::timeout) {
    FAIL() << "Test timed out after " << timeout.count() << " seconds.";
  } else {
    ASSERT_TRUE(test_future.get().empty());
  }
}
