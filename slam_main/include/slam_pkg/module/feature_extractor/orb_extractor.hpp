//
// Created by park on 24. 6. 26.
//

#pragma once

#include "slam_pkg/type/geometry/odometry.hpp"
#include "slam_pkg/type/sensor/camera_info.hpp"
#include "slam_pkg/type/sensor/image.hpp"

namespace Slam::module {

class OrbExtractor {
 public:
  void receive_data([[maybe_unused]] const std::string& topic, CameraInfo&& data) {
    if (info_.has_value()) return;

    info_ = std::move(data);
  }

  void receive_data(const std::string& topic, Image&& data) {
    if (topic_.empty()) topic_ = topic;

    image_ = std::move(data);

    if (is_ready()) process();
  }

  void receive_data([[maybe_unused]] const std::string& topic, Odometry&& data) {
    if (odom_.header.stamp == 0) prev_odom_ = data;
    odom_ = std::move(data);
  }

  bool is_ready() {
    // early return: has value
    if (!info_.has_value() || !image_.has_value()) return false;

    // early return: odometry
    auto pose_diff = prev_odom_.pose_w_cov.pose - odom_.pose_w_cov.pose;
    if (pose_diff.p.norm() < 0.01 && pose_diff.q.angle_to({}) < 0.01) return false;

    prev_odom_ = odom_;
    return true;
  }

  void process() {
    // feature detect
    auto feature_detector = cv::ORB::create();
    feature_detector->detectAndCompute(image_->value, cv::Mat(), kps_, desc_);

    if (!prev_image_.has_value()) {
      prev_image_ = std::move(image_);
      prev_kps_ = std::move(kps_);
      prev_desc_ = std::move(desc_);
      return;
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::DMatch> good_knn_matches;
    auto knn_matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    knn_matcher.knnMatch(desc_, prev_desc_, knn_matches, 2);

    for (const auto& match : knn_matches) {
      if (match.size() != 2) continue;

      if (match[0].distance < knn_dist_ratio_threshold_ * match[1].distance) {
        good_knn_matches.push_back(match[0]);
      }
    }

    cv::Mat img_knn;
    drawMatches(image_->value, kps_, prev_image_->value, prev_kps_, good_knn_matches, img_knn,
                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow(topic_, img_knn);
    cv::waitKey(1);

    prev_image_ = std::move(image_);
    prev_kps_ = std::move(kps_);
    prev_desc_ = std::move(desc_);
  }

 private:
  std::string topic_{};
  std::optional<CameraInfo> info_{};
  std::optional<Image> image_{}, prev_image_{};

  std::vector<cv::KeyPoint> kps_{}, prev_kps_{};
  cv::Mat desc_{}, prev_desc_{};

  Odometry odom_{}, prev_odom_{};

  /// parameters
  double knn_dist_ratio_threshold_{0.8};
  double odom_dist_threshold_{0.1};
};

}  // namespace Slam::module