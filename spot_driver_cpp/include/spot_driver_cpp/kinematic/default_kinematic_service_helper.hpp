// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/kinematic/kinematic_service_helper.hpp>

#include <memory>
#include <string>

namespace spot_ros2::kinematic {

class DefaultKinematicServiceHelper : public KinematicServiceHelper {
 public:
  explicit DefaultKinematicServiceHelper(std::shared_ptr<rclcpp::Node> node);
  std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>> create_service(
      std::string service_name, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                   std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                    callback) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace spot_ros2::kinematic
