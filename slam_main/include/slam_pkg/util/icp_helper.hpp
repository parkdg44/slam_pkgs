/**
 * @file      icp_helper.hpp
 * @author    Donggyu Park (pksdd124@gmail.com)
 * @brief     LidarOdometry class header
 * @date      2025-11-28
 *
 * @copyright Copyright (c) 2025 Donggyu Park. All rights reserved.
 */

#pragma once

#include <Eigen/Geometry>
#include <functional>
#include <numeric>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include "lie_group_functions.hpp"

namespace slam::util {

class IcpHelper {
  // reference: https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Registration.cpp

  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
  using LinearSystem = std::pair<Matrix6d, Vector6d>;
  using NearestSearchResult = std::pair<Eigen::Vector3d, double>;

 public:
  static Correspondences find_correspondence(
      const std::vector<Eigen::Vector3d>& points,
      const std::function<NearestSearchResult(const Eigen::Vector3d&)>& find_closest_neighbor,
      const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator>& r) {
          std::for_each(r.begin(), r.end(), [&](const auto& point) {
            const auto& [closest_neighbor, distance] = find_closest_neighbor(point);
            if (distance < max_correspondance_distance) {
              correspondences.emplace_back(point, closest_neighbor);
            }
          });
        });
    return correspondences;
  }

  // TODO: VoxelHashMap class implementation needed
  // static Correspondences find_correspondence(const std::vector<Eigen::Vector3d>& points,
  //                                            const kiss_icp::VoxelHashMap& voxel_map,
  //                                            const double max_correspondance_distance) {
  //   using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
  //   Correspondences correspondences;
  //   correspondences.reserve(points.size());
  //   tbb::parallel_for(
  //       // Range
  //       tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
  //       [&](const tbb::blocked_range<points_iterator>& r) {
  //         std::for_each(r.begin(), r.end(), [&](const auto& point) {
  //           const auto& [closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
  //           if (distance < max_correspondance_distance) {
  //             correspondences.emplace_back(point, closest_neighbor);
  //           }
  //         });
  //       });
  //   return correspondences;
  // }

  static LinearSystem BuildLinearSystem(const Correspondences& correspondences,
                                        const double kernel_scale) {
    static auto compute_jacobian_and_residual = [](const auto& correspondence) {
      const auto& [source, target] = correspondence;
      const Eigen::Vector3d residual = source - target;
      Eigen::Matrix<double, 3, 6> J_r;
      J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      J_r.block<3, 3>(0, 3) = -1.0 * hat(source);
      return std::make_tuple(J_r, residual);
    };

    static auto sum_linear_systems = [](LinearSystem a, const LinearSystem& b) {
      a.first += b.first;
      a.second += b.second;
      return a;
    };

    static auto square = [](const double& x) { return x * x; };

    auto GM_weight = [&](const double& residual2) {
      return square(kernel_scale) / square(kernel_scale + residual2);
    };

    using correspondence_iterator = Correspondences::const_iterator;
    const auto& [JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem(Matrix6d::Zero(), Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator>& r, LinearSystem J) -> LinearSystem {
          return std::transform_reduce(
              r.begin(), r.end(), J, sum_linear_systems, [&](const auto& correspondence) {
                const auto& [J_r, residual] = compute_jacobian_and_residual(correspondence);
                const double w = GM_weight(residual.squaredNorm());
                return LinearSystem(J_r.transpose() * w * J_r,        // JTJ
                                    J_r.transpose() * w * residual);  // JTr
              });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);

    return {JTJ, JTr};
  }

  Eigen::Isometry3d align(
      const std::vector<Eigen::Vector3d>& frame,
      const std::function<NearestSearchResult(const Eigen::Vector3d&)>& find_closest_neighbor,
      const Eigen::Isometry3d& initial_guess, const double max_distance,
      const double kernel_scale) {
    // if (map.empty()) return initial_guess;

    // Equation (9)
    std::vector<Eigen::Vector3d> source = frame;
    for (auto& point : source) point = initial_guess * point;

    static constexpr double convergence_criterion = 1e-4;
    static constexpr int max_num_iterations = 20;

    // ICP-loop
    Eigen::Isometry3d T_icp = Eigen::Isometry3d::Identity();
    for (int j = 0; j < max_num_iterations; ++j) {
      // Equation (10)
      const auto correspondences = find_correspondence(source, find_closest_neighbor, max_distance);
      // Equation (11)
      const auto& [JTJ, JTr] = BuildLinearSystem(correspondences, kernel_scale);
      const Vector6d dx = JTJ.ldlt().solve(-JTr);
      const Eigen::Isometry3d estimation = Se3::Exp(dx);
      // Equation (12)
      for (auto& point : source) point = estimation * point;
      // Update iterations
      T_icp = estimation * T_icp;
      // Termination criteria
      if (dx.norm() < convergence_criterion) break;
    }
    // Spit the final transformation
    return T_icp * initial_guess;
  }

  // TODO: VoxelHashMap class implementation needed
  // Sophus::SE3d align(const std::vector<Eigen::Vector3d>& frame, const VoxelHashMap& voxel_map,
  //                    const Sophus::SE3d& initial_guess, const double max_distance,
  //                    const double kernel_scale) {
  //   if (voxel_map.Empty()) return initial_guess;

  //   // Equation (9)
  //   std::vector<Eigen::Vector3d> source = frame;
  //   TransformPoints(initial_guess, source);

  //   // ICP-loop
  //   Sophus::SE3d T_icp = Sophus::SE3d();
  //   for (int j = 0; j < max_num_iterations_; ++j) {
  //     // Equation (10)
  //     const auto correspondences = DataAssociation(source, voxel_map, max_distance);
  //     // Equation (11)
  //     const auto& [JTJ, JTr] = BuildLinearSystem(correspondences, kernel_scale);
  //     const Vector6d dx = JTJ.ldlt().solve(-JTr);
  //     const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
  //     // Equation (12)
  //     TransformPoints(estimation, source);
  //     // Update iterations
  //     T_icp = estimation * T_icp;
  //     // Termination criteria
  //     if (dx.norm() < convergence_criterion_) break;
  //   }
  //   // Spit the final transformation
  //   return T_icp * initial_guess;
  // }
};
}  // namespace slam::util