//
// Created by park on 24. 4. 3.
//

#pragma once

#include <functional>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <shared_mutex>
#include <vector>

#include "slam_pkg/type/geometry/header.hpp"
#include "slam_pkg/type/geometry/pose.hpp"
#include "slam_pkg/type/geometry/quaternion.hpp"
#include "slam_pkg/type/geometry/twist.hpp"
#include "slam_pkg/type/geometry/vector.hpp"

namespace Slam::ros2 {

static inline builtin_interfaces::msg::Time to_ros(Time const& data) {
  return rclcpp::Time(data.value);
}

static inline std_msgs::msg::Header to_ros(Header const& data) {
  std_msgs::msg::Header output;
  output.stamp = to_ros(data.stamp);
  output.frame_id = data.ref_frame_id;
  return output;
}

template <typename ROS_T = geometry_msgs::msg::Point>
static inline ROS_T to_ros(Vector const& data);

template <>
inline geometry_msgs::msg::Point to_ros(Vector const& data) {
  geometry_msgs::msg::Point output;
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

template <>
inline geometry_msgs::msg::Vector3 to_ros(Vector const& data) {
  geometry_msgs::msg::Vector3 output;
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

static inline geometry_msgs::msg::Quaternion to_ros(Quaternion const& data) {
  geometry_msgs::msg::Quaternion output;
  output.w = data.w();
  output.x = data.x();
  output.y = data.y();
  output.z = data.z();
  return output;
}

static inline geometry_msgs::msg::Pose to_ros(Pose const& data) {
  geometry_msgs::msg::Pose output;
  output.position = to_ros(data.translation());
  output.orientation = to_ros(data.rotation());
  return output;
}

static inline geometry_msgs::msg::PoseStamped to_ros(PoseStamped const& data) {
  geometry_msgs::msg::PoseStamped output;
  output.header = to_ros(data.header);
  output.pose = to_ros(data.pose);
  return output;
}

static inline geometry_msgs::msg::Twist to_ros(Twist const& data) {
  geometry_msgs::msg::Twist output;
  output.angular = to_ros<geometry_msgs::msg::Vector3>(data.angular);
  output.linear = to_ros<geometry_msgs::msg::Vector3>(data.linear);
  return output;
}

static inline Time from_ros(builtin_interfaces::msg::Time const& data) {
  Time output;
  output.value = rclcpp::Time(data).nanoseconds();
  return output;
}

static inline Header from_ros(std_msgs::msg::Header const& data) {
  Header output;
  output.stamp = from_ros(data.stamp);
  output.ref_frame_id = data.frame_id;
  return output;
}

static inline Vector from_ros(geometry_msgs::msg::Point const& data) {
  Vector output;
  output.x() = data.x;
  output.y() = data.y;
  output.z() = data.z;
  return output;
}

static inline Vector from_ros(geometry_msgs::msg::Vector3 const& data) {
  Vector output;
  output.x() = data.x;
  output.y() = data.y;
  output.z() = data.z;
  return output;
}

static inline Quaternion from_ros(geometry_msgs::msg::Quaternion const& data) {
  Quaternion output;
  output.w() = data.w;
  output.x() = data.x;
  output.y() = data.y;
  output.z() = data.z;
  return output;
}

static inline Pose from_ros(geometry_msgs::msg::Pose const& data) {
  Pose output;
  output.translation() = from_ros(data.position);
  output.rotation() = from_ros(data.orientation);
  return output;
}

static inline PoseStamped from_ros(geometry_msgs::msg::PoseStamped const& data) {
  PoseStamped output;
  output.header = from_ros(data.header);
  output.pose = from_ros(data.pose);
  return output;
}

static inline Twist from_ros(geometry_msgs::msg::Twist const& data) {
  Twist output;
  output.angular.x() = data.angular.x;
  output.angular.y() = data.angular.y;
  output.angular.z() = data.angular.z;
  output.linear.x() = data.linear.x;
  output.linear.y() = data.linear.y;
  output.linear.z() = data.linear.z;
  return output;
}

template <typename SlamT>
std::function<void(std::string const&, SlamT const&)> ros_pub_factory(rclcpp::Node& node,
                                                                      rclcpp::QoS const& qos) {
  // [추가된 로직] 이 팩토리 함수가 해당 타입(SlamT)에 대해 이미 호출되었는지 확인
  static bool is_instantiated = false;
  if (is_instantiated) {
    throw std::runtime_error(
        "ros_pub_factory called more than once for this type. It is designed for Singleton usage.");
  }
  is_instantiated = true;

  using RosT = decltype(to_ros(std::declval<SlamT>()));

  static_assert(std::is_same_v<decltype(to_ros(std::declval<const SlamT&>())), RosT>,
                "to_ros(SlamT) must be implemented for this RosT type.");

  return [&node, &qos](std::string const& topic, SlamT const& data) {
    static std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> map_pub{};

    if (map_pub.find(topic) == map_pub.end()) {
      map_pub[topic] = node.create_publisher<RosT>(topic, qos);
    }

    auto pub_data = std::static_pointer_cast<rclcpp::Publisher<RosT>>(map_pub[topic]);
    pub_data->publish(to_ros(data));
  };
}

template <typename SlamT>
std::function<void(std::string const&, std::function<void(SlamT const&) &&>)> ros_sub_factory(
    rclcpp::Node& node, rclcpp::QoS const& qos) {
  // [추가된 로직] 이 팩토리 함수가 해당 타입(SlamT)에 대해 이미 호출되었는지 확인
  static bool is_instantiated = false;
  if (is_instantiated) {
    throw std::runtime_error(
        "ros_sub_factory called more than once for this type. It is designed for Singleton usage.");
  }
  is_instantiated = true;

  using RosT = decltype(to_ros(std::declval<SlamT>()));

  static_assert(std::is_same_v<decltype(from_ros(std::declval<RosT>())), SlamT>,
                "from_ros(RosT) must be implemented for this SlamT type.");

  return [&node, &qos](std::string const& topic, std::function<void(SlamT const&)>&& callback) {
    static std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> map_sub{};
    static std::unordered_map<std::string, std::vector<std::function<void(SlamT const&)>>> map_cb{};
    static std::unordered_map<std::string, std::shared_mutex> map_mtx{};

    if (map_sub.find(topic) == map_sub.end()) {
      map_sub[topic] = node.create_subscription<RosT>(
          topic, qos, [topic, mtx = std::ref(map_mtx[topic])](const RosT& data) {
            std::shared_lock lock(mtx.get());
            for (auto& cb : map_cb[topic]) cb(from_ros(data));
          });
    }

    {  // critical section to add callback
      std::unique_lock lock(map_mtx[topic]);
      map_cb[topic].push_back(std::move(callback));
    }
  };
}

}  // namespace Slam::ros2
