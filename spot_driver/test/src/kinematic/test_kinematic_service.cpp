// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <memory>

#include <spot_driver/api/kinematic_api.hpp>
#include <spot_driver/kinematic/kinematic_service.hpp>
#include <spot_driver/mock/mock_kinematic_api.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>

#include <tl_expected/expected.hpp>

namespace spot_ros2::kinematic::test {

using ::testing::_;
using ::testing::Return;

class MockMiddlewareHandle : public KinematicService::MiddlewareHandle {
 public:
  MOCK_METHOD((void), createService,
              (std::string serviceName, std::function<void(const std::shared_ptr<GetInverseKinematicSolutions::Request>,
                                                           std::shared_ptr<GetInverseKinematicSolutions::Response>)>
                                            callback),
              (override));
};

/**
 * Test the service initialization.
 */
TEST(TestKinematicService, initialize) {
  auto ik_api = std::make_unique<spot_ros2::test::MockKinematicApi>();
  auto logger = std::make_shared<spot_ros2::test::MockLoggerInterface>();
  auto middleware = std::make_unique<MockMiddlewareHandle>();
  EXPECT_CALL(*middleware, createService(_, _)).Times(1);

  auto ik_service = std::make_unique<KinematicService>(std::move(ik_api), logger, std::move(middleware));
  ik_service->initialize();
}

/**
 * Test a happy path behavior when an IK solution is received.
 */
TEST(TestKinematicService, getSolutions) {
  // GIVEN an instance of the Spot IK API.

  auto ik_api = std::make_unique<spot_ros2::test::MockKinematicApi>();

  // GIVEN the IK API will succeed and return a representative nominal result
  // the first time it is called.
  // THEN a single request is sent to Spot's inverse kinematics API

  InverseKinematicsResponse fake_response;
  fake_response.set_status(bosdyn::api::spot::InverseKinematicsResponse_Status_STATUS_OK);
  tl::expected<InverseKinematicsResponse, std::string> fake_result(fake_response);

  EXPECT_CALL(*ik_api, getSolutions(_)).WillOnce(Return(fake_result));

  auto logger = std::make_shared<spot_ros2::test::MockLoggerInterface>();
  auto middleware = std::make_unique<MockMiddlewareHandle>();

  auto ik_service = std::make_unique<KinematicService>(std::move(ik_api), logger, std::move(middleware));
  ik_service->initialize();

  // WHEN an IK request is made through the IK service.

  auto request = std::make_shared<GetInverseKinematicSolutions::Request>();
  auto response = std::make_shared<GetInverseKinematicSolutions::Response>();
  ik_service->getSolutions(request, response);

  // THEN the IK response indicates that the request succeeds.

  ASSERT_EQ(response->response.status.value, bosdyn_spot_api_msgs::msg::InverseKinematicsResponseStatus::STATUS_OK);
}

/**
 * Test the behavior of the service when an exception is thrown during an IK request.
 */
TEST(TestKinematicService, getSolutionsException) {
  auto ik_api = std::make_unique<spot_ros2::test::MockKinematicApi>();

  EXPECT_CALL(*ik_api, getSolutions(_)).WillOnce(Return(tl::make_unexpected("Some error")));

  auto logger = std::make_shared<spot_ros2::test::MockLoggerInterface>();
  auto middleware = std::make_unique<MockMiddlewareHandle>();

  auto ik_service = std::make_unique<KinematicService>(std::move(ik_api), logger, std::move(middleware));
  ik_service->initialize();

  auto request = std::make_shared<GetInverseKinematicSolutions::Request>();
  auto response = std::make_shared<GetInverseKinematicSolutions::Response>();
  ik_service->getSolutions(request, response);

  ASSERT_EQ(response->response.status.value,
            bosdyn_spot_api_msgs::msg::InverseKinematicsResponseStatus::STATUS_UNKNOWN);
}

}  // namespace spot_ros2::kinematic::test
