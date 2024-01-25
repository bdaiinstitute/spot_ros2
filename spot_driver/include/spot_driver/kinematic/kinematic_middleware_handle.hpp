// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/kinematic/kinematic_service.hpp>

#include <memory>
#include <rclcpp/node.hpp>
#include <string>

namespace spot_ros2::kinematic {

class KinematicMiddlewareHandle : public KinematicService::MiddlewareHandle {
 public:
  explicit KinematicMiddlewareHandle(std::shared_ptr<rclcpp::Node> node);
  void createService(std::string service_name,
                     std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                        std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                         callback) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>> service_;
};
}  // namespace spot_ros2::kinematic
