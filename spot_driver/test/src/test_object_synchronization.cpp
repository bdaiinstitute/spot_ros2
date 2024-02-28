// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/api/world_object.pb.h>
#include <gmock/gmock-actions.h>
#include <gmock/gmock-matchers.h>
#include <gmock/gmock-spec-builders.h>
#include <gmock/gmock.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_node_interface.hpp>
#include <spot_driver/mock/mock_state_client.hpp>
#include <spot_driver/mock/mock_state_publisher_middleware_handle.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_tf_listener_interface.hpp>
#include <spot_driver/mock/mock_time_sync_api.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/types.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tl_expected/expected.hpp>
#include <utility>
#include "spot_driver/mock/mock_robot_model_interface.hpp"
#include "spot_driver/mock/mock_world_object_client.hpp"

namespace {
using ::testing::_;
using ::testing::AllOf;
using ::testing::HasSubstr;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Property;
using ::testing::Return;
using ::testing::Unused;

MATCHER(RequestAddsObject, "") {
  return testing::ExplainMatchResult(
      testing::Eq(arg.mutation().action()),
      ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_ADD, result_listener);
}

MATCHER(RequestChangesObject, "") {
  return testing::ExplainMatchResult(
      testing::Eq(arg.mutation().action()),
      ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_CHANGE, result_listener);
}

constexpr auto kErrorMessage = "Some error message.";
}  // namespace

namespace spot_ros2::test {
class ObjectSynchronizerTest : public ::testing::Test {
 public:
  void SetUp() override {
    mock_node_interface = std::make_unique<MockNodeInterface>();
    fake_parameter_interface = std::make_unique<FakeParameterInterface>();
    fake_parameter_interface->spot_name = "MyRobot";
    mock_logger_interface = std::make_unique<MockLoggerInterface>();
    mock_tf_interface = std::make_unique<MockTfInterface>();
    mock_tf_listener_interface = std::make_unique<MockTfListenerInterface>();
    mock_robot_model_interface = std::make_unique<MockRobotModelInterface>();
    mock_timer_interface = std::make_unique<MockTimerInterface>();
    mock_state_client = std::make_shared<MockStateClient>();
    mock_world_object_client = std::make_shared<MockWorldObjectClient>();
    mock_time_sync_api = std::make_shared<MockTimeSyncApi>();
  }

  void createObjectSynchronizer() {
    object_synchronizer = std::make_unique<ObjectSynchronizer>(
        mock_state_client, mock_world_object_client, mock_time_sync_api, std::move(fake_parameter_interface),
        std::move(mock_logger_interface), std::move(mock_tf_listener_interface), std::move(mock_timer_interface),
        std::move(mock_robot_model_interface));
  }

  void setInternalTimerCallback() const {
    auto* timer_interface_ptr = mock_timer_interface.get();
    ON_CALL(*timer_interface_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
      timer_interface_ptr->onSetTimer(cb);
    });
  }

  std::unique_ptr<MockNodeInterface> mock_node_interface;
  std::unique_ptr<FakeParameterInterface> fake_parameter_interface;
  std::unique_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<MockTfInterface> mock_tf_interface;
  std::unique_ptr<MockTfListenerInterface> mock_tf_listener_interface;
  std::unique_ptr<MockRobotModelInterface> mock_robot_model_interface;

  std::unique_ptr<MockTimerInterface> mock_timer_interface;
  std::shared_ptr<MockStateClient> mock_state_client;
  std::shared_ptr<MockWorldObjectClient> mock_world_object_client;
  std::shared_ptr<MockTimeSyncApi> mock_time_sync_api;

  std::unique_ptr<ObjectSynchronizer> object_synchronizer;
};

// bosdyn::api::RobotState makeRobotState(const bool has_valid_transforms = true) {
//   google::protobuf::Timestamp timestamp;
//   timestamp.set_seconds(100);
//   timestamp.set_nanos(0);
//   bosdyn::api::RobotState out;
//   addAcquisitionTimestamp(out.mutable_kinematic_state(), timestamp);

//   if (has_valid_transforms) {
//     addTransform(out.mutable_kinematic_state()->mutable_transforms_snapshot(), "some_frame", "some_other_frame", 0,
//     0,
//                  0, 1, 0, 0, 0);
//   } else {
//     out.mutable_kinematic_state()->mutable_transforms_snapshot()->Clear();
//   }
//   return out;
// }

TEST_F(ObjectSynchronizerTest, InitSucceeds) {
  // THEN the timer interface's setTimer function is called once with the expected timer period
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0}, _));

  // WHEN a robot state publisher is constructed
  createObjectSynchronizer();

  // // WHEN the timer callback is triggered
  // timer_interface_ptr->trigger();
}

TEST_F(ObjectSynchronizerTest, AddFrameAsWorldObject) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  auto* timer_interface_ptr = mock_timer_interface.get();
  ON_CALL(*timer_interface_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  constexpr auto kExternalFrameId = "some_external_frame";
  auto* tf_listener_interface_ptr = mock_tf_listener_interface.get();
  EXPECT_CALL(*tf_listener_interface_ptr, getAllFrameNames)
      .Times(1)
      .WillOnce(Return(std::vector<std::string>{"MyRobot/body", kExternalFrameId}));
  ON_CALL(*tf_listener_interface_ptr, lookupTransform)
      .WillByDefault(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  // auto* robot_interface_ptr = mock_robot_model_interface.get();
  // ON_CALL(*robot_interface_ptr, getFrameIds).WillByDefault(Return(tl::expected<std::set<std::string>,
  // std::string>{std::set<std::string>{"MyRobot/body"}}));

  ::bosdyn::api::ListWorldObjectResponse list_apriltags_response;

  // ::bosdyn::api::WorldObject object;
  // object.set_name("my_object");
  // object.mutable_transforms_snapshot()
  ::bosdyn::api::ListWorldObjectResponse list_objects_response;
  // auto* object = list_objects_response.add_world_objects();
  // object->set_name("my_object");

  auto* world_object_client_interface_ptr = mock_world_object_client.get();
  EXPECT_CALL(*world_object_client_interface_ptr, listWorldObjects)
      .WillOnce(Return(list_apriltags_response))
      .WillOnce(Return(list_objects_response));

  EXPECT_CALL(*world_object_client_interface_ptr, mutateWorldObject(RequestAddsObject())).Times(1);

  // WHEN a robot state publisher is constructed
  createObjectSynchronizer();

  // // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}

// TEST_F(StatePublisherTest, PublishCallbackTriggers) {
//   // THEN the timer interface's setTimer function is called once and the timer_callback is set
//   auto* timer_interface_ptr = mock_timer_interface.get();
//   EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
//     timer_interface_ptr->onSetTimer(cb);
//   });

//   {
//     InSequence seq;
//     // GIVEN the robot state will contain transforms
//     // THEN we request the latest clock skew from the Spot interface
//     EXPECT_CALL(*mock_time_sync_api, getClockSkew).Times(1).WillRepeatedly(Return(google::protobuf::Duration()));
//     // AND THEN we request the robot state from the Spot interface
//     EXPECT_CALL(*mock_state_client_interface, getRobotState)
//         .WillOnce(Return(tl::expected<bosdyn::api::RobotState, std::string>{makeRobotState(true)}));
//     // AND THEN we publish the robot state to the appropriate topics
//     EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(1);
//     // AND THEN the robot transforms are published to TF
//     EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(1);
//   }

//   // GIVEN a robot_state_publisher
//   robot_state_publisher = std::make_unique<StatePublisher>(
//       mock_state_client_interface, mock_time_sync_api, std::move(mock_middleware_handle),
//       std::move(fake_parameter_interface), std::move(mock_logger_interface), std::move(mock_tf_interface),
//       std::move(mock_timer_interface));

//   // WHEN the timer callback is triggered
//   timer_interface_ptr->trigger();
// }

// TEST_F(StatePublisherTest, PublishCallbackTriggersNoTfData) {
//   // THEN the timer interface's setTimer function is called once and the timer_callback is set
//   auto* timer_interface_ptr = mock_timer_interface.get();
//   EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
//     timer_interface_ptr->onSetTimer(cb);
//   });

//   {
//     InSequence seq;
//     // GIVEN the robot state does not contain any transforms
//     // THEN we request the latest clock skew from the Spot interface
//     EXPECT_CALL(*mock_time_sync_api, getClockSkew).Times(1).WillRepeatedly(Return(google::protobuf::Duration()));
//     // AND THEN we request the robot state from the Spot interface
//     EXPECT_CALL(*mock_state_client_interface, getRobotState)
//         .WillOnce(Return(tl::expected<bosdyn::api::RobotState, std::string>(makeRobotState(false))));
//     // AND THEN we publish the robot state to the appropriate topics
//     EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(1);
//   }

//   // THEN no transforms are published to TF
//   EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(0);

//   // GIVEN a robot_state_publisher
//   robot_state_publisher = std::make_unique<StatePublisher>(
//       mock_state_client_interface, mock_time_sync_api, std::move(mock_middleware_handle),
//       std::move(fake_parameter_interface), std::move(mock_logger_interface), std::move(mock_tf_interface),
//       std::move(mock_timer_interface));

//   // WHEN the timer callback is triggered
//   timer_interface_ptr->trigger();
// }

// TEST_F(StatePublisherTest, PublishCallbackTriggersFailGetRobotState) {
//   // THEN the timer interface's setTimer function is called once and the timer_callback is set
//   auto* timer_interface_ptr = mock_timer_interface.get();
//   EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
//     timer_interface_ptr->onSetTimer(cb);
//   });

//   auto* logger_interface_ptr = mock_logger_interface.get();
//   {
//     InSequence seq;
//     // THEN we request the latest clock skew from the Spot interface
//     EXPECT_CALL(*mock_time_sync_api, getClockSkew).Times(1).WillRepeatedly(Return(google::protobuf::Duration()));

//     // GIVEN the request to retrieve the robot state will fail
//     // THEN we request the robot state from the Spot interface
//     EXPECT_CALL(*mock_state_client_interface, getRobotState)
//         .Times(1)
//         .WillRepeatedly(Return(tl::make_unexpected(kErrorMessage)));
//     // THEN an error message is logged
//     EXPECT_CALL(*logger_interface_ptr, logError(HasSubstr(kErrorMessage))).Times(1);
//   }

//   // THEN we do not publish a robot state
//   EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(0);
//   // THEN we do not publish to TF
//   EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(0);

//   // GIVEN a robot_state_publisher
//   robot_state_publisher = std::make_unique<StatePublisher>(
//       mock_state_client_interface, mock_time_sync_api, std::move(mock_middleware_handle),
//       std::move(fake_parameter_interface), std::move(mock_logger_interface), std::move(mock_tf_interface),
//       std::move(mock_timer_interface));

//   // WHEN the timer callback is triggered
//   timer_interface_ptr->trigger();
// }

// TEST_F(StatePublisherTest, PublishCallbackTriggersFailGetClockSkew) {
//   // THEN the timer interface's setTimer function is called once and the timer_callback is set
//   auto* timer_interface_ptr = mock_timer_interface.get();
//   EXPECT_CALL(*timer_interface_ptr, setTimer).Times(1).WillOnce([&](Unused, const std::function<void()>& cb) {
//     timer_interface_ptr->onSetTimer(cb);
//   });

//   auto* logger_interface_ptr = mock_logger_interface.get();
//   {
//     InSequence seq;
//     // GIVEN the request to retrieve the clock skew state fail
//     // THEN we request the latest clock skew from the Spot interface
//     EXPECT_CALL(*mock_time_sync_api,
//     getClockSkew).Times(1).WillRepeatedly(Return(tl::make_unexpected(kErrorMessage)));
//     // AND THEN an error message is logged
//     EXPECT_CALL(*logger_interface_ptr, logError(HasSubstr(kErrorMessage))).Times(1);
//   }

//   // THEN we do not publish a robot state
//   EXPECT_CALL(*mock_middleware_handle, publishRobotState).Times(0);
//   // THEN we do not publish to TF
//   EXPECT_CALL(*mock_tf_interface, sendDynamicTransforms).Times(0);

//   // GIVEN a robot_state_publisher
//   robot_state_publisher = std::make_unique<StatePublisher>(
//       mock_state_client_interface, mock_time_sync_api, std::move(mock_middleware_handle),
//       std::move(fake_parameter_interface), std::move(mock_logger_interface), std::move(mock_tf_interface),
//       std::move(mock_timer_interface));

//   // WHEN the timer callback is triggered
//   timer_interface_ptr->trigger();
// }
}  // namespace spot_ros2::test
