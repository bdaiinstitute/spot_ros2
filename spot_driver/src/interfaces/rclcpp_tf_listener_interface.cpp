// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <spot_driver/interfaces/rclcpp_tf_listener_interface.hpp>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {
RclcppTfListenerInterface::RclcppTfListenerInterface(const std::shared_ptr<rclcpp::Node>& node)
    : buffer_{node->get_clock()}, listener_{buffer_} {}

std::vector<std::string> RclcppTfListenerInterface::getAllFrameNames() const {
  return buffer_.getAllFrameNames();
}

tl::expected<geometry_msgs::msg::TransformStamped, std::string> RclcppTfListenerInterface::lookupTransform(
    const std::string& parent, const std::string& child, const rclcpp::Time& timepoint,
    const rclcpp::Duration& timeout) const {
  try {
    return buffer_.lookupTransform(child, parent, tf2::TimePointZero);
    // return buffer_.lookupTransform(child, parent, timepoint, timeout);
  } catch (const tf2::LookupException& e) {
    return tl::make_unexpected(e.what());
  } catch (const tf2::ConnectivityException& e) {
    return tl::make_unexpected(e.what());
  } catch (const tf2::ExtrapolationException& e) {
    return tl::make_unexpected(e.what());
  } catch (const tf2::InvalidArgumentException& e) {
    return tl::make_unexpected(e.what());
  }
}
}  // namespace spot_ros2
