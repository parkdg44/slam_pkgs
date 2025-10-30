//
// Created by park on 24. 6. 12.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace Slam::ros2::util {

template <typename T>
T declare_and_get_parameter(rclcpp::Node* node, std::string const& name, T default_value = {}) {
  node->declare_parameter(name, default_value);

  if constexpr (std::is_same_v<T, std::string>) {
    return node->get_parameter(name).as_string();
  }
  if constexpr (std::is_same_v<T, bool>) {
    return node->get_parameter(name).as_bool();
  }
  if constexpr (std::is_same_v<T, int>) {
    return node->get_parameter(name).as_int();
  }
  if constexpr (std::is_same_v<T, double>) {
    return node->get_parameter(name).as_double();
  }
  if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return node->get_parameter(name).as_string_array();
  }
  if constexpr (std::is_same_v<T, std::vector<bool>>) {
    return node->get_parameter(name).as_bool_array();
  }
  if constexpr (std::is_same_v<T, std::vector<int>>) {
    return node->get_parameter(name).as_integer_array();
  }
  if constexpr (std::is_same_v<T, std::vector<double>>) {
    return node->get_parameter(name).as_double_array();
  }
  throw std::runtime_error(std::string(typeid(T).name()) + " type can't be used.");
}

}  // namespace Slam::ros2::util