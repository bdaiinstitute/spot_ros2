// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <memory>

#include <spot_driver_cpp/api/kinematic_api.hpp>
#include <spot_driver_cpp/kinematic/kinematic_service.hpp>
#include <spot_driver_cpp/mock/mock_kinematic_api.hpp>
#include <spot_driver_cpp/mock/mock_logger_interface.hpp>

namespace spot_ros2::kinematic::test {

using ::testing::_;

class MockMiddlewareHandle : public KinematicService::MiddlewareHandle {
 public:
  MOCK_METHOD((std::shared_ptr<rclcpp::Service<GetInverseKinematicSolutions>>), create_service,
              (std::string service_name,
               std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                  std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                   callback),
              (override));
};

TEST(TestKinematicService, initialize) {
  auto ik_api = std::make_unique<spot_ros2::test::MockKinematicApi>();
  auto logger = std::make_shared<spot_ros2::test::MockLoggerInterface>();
  auto middleware = std::make_unique<MockMiddlewareHandle>();
  EXPECT_CALL(*middleware, create_service(_, _)).Times(1);

  auto ik_service = std::make_unique<KinematicService>(std::move(ik_api), logger, std::move(middleware));
  ik_service->initialize();
}

}  // namespace spot_ros2::kinematic::test
