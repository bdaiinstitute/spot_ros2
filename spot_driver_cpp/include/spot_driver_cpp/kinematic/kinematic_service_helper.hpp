// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_msgs/srv/get_inverse_kinematic_solutions.hpp>

#include <rclcpp/node.hpp>

#include <functional>
#include <memory>
#include <string>

namespace spot_ros2::kinematic {

using spot_msgs::srv::GetInverseKinematicSolutions;

class KinematicServiceHelper {
 public:
  virtual ~KinematicServiceHelper() {}
  virtual std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>> create_service(
      std::string service_name, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                   std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                    callback) = 0;
};
}  // namespace spot_ros2