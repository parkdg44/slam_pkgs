//
// Created by park on 24. 6. 26.
//

#pragma once

#include "slam_pkg/type/sensor/camera_info.hpp"
#include "slam_pkg/type/sensor/image.hpp"

namespace Slam::module {

class OrbExtractor {
 public:
  void receive_data(const std::string& topic, CameraInfo&& data) {
    if (info_.has_value()) return;
    info_ = std::move(data);
  }

  void receive_data(const std::string& topic, Image&& data) {
    image_ = std::move(data);

    if (is_ready()) process();
  }

  bool is_ready() {
    // early return: has value
    if (!info_.has_value() || !image_.has_value()) return false;

    // early return: span check
    // Time span = info_->header.stamp - image_->header.stamp;
    // if (std::fabs(span.to_ms()) > 100.0) return false;

    return true;
  }

  void process() {
    const cv::Mat& cv_img = image_->value;
    std::vector<cv::KeyPoint> key_points;
    cv::Mat descriptor;

    auto feature_detector = cv::ORB::create();

    feature_detector->detectAndCompute(cv_img, cv::Mat(), key_points, descriptor);

    cv::Mat output;
    cv::drawKeypoints(cv_img, key_points, output);
    cv::imshow(image_->header.ref_frame_id, output);
    cv::waitKey(1);
  }

 private:
  std::optional<CameraInfo> info_;
  std::optional<Image> image_, prev_image_;

  std::vector<cv::KeyPoint> key_points_{};
};

}  // namespace Slam::module