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
#include <spot_driver/mock/mock_robot_model_interface.hpp>
#include <spot_driver/mock/mock_state_client.hpp>
#include <spot_driver/mock/mock_state_publisher_middleware_handle.hpp>
#include <spot_driver/mock/mock_tf_interface.hpp>
#include <spot_driver/mock/mock_tf_listener_interface.hpp>
#include <spot_driver/mock/mock_time_sync_api.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>
#include <spot_driver/mock/mock_world_object_client.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/types.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tl_expected/expected.hpp>
#include <utility>

namespace {
using ::testing::_;
using ::testing::AllOf;
using ::testing::Eq;
using ::testing::ExplainMatchResult;
using ::testing::Field;
using ::testing::HasSubstr;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Property;
using ::testing::Return;
using ::testing::StrEq;
using ::testing::Unused;

MATCHER(MutationAddsObject, "") {
  return ExplainMatchResult(Eq(arg.mutation().action()),
                            ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_ADD,
                            result_listener);
}

MATCHER(MutationChangesObject, "") {
  return testing::ExplainMatchResult(
      testing::Eq(arg.mutation().action()),
      ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_CHANGE, result_listener);
}

MATCHER_P(MutationTargetsObjectWhoseNameIs, object_name, "") {
  return testing::ExplainMatchResult(StrEq(arg.mutation().object().name()), object_name, result_listener);
}

// MATCHER_P(TransformEq, transform, "")
// {
//   return testing::ExplainMatchResult(Eq(arg), transform, result_listener);
// }

// MATCHER_P(MutationObjectTransformSnapshotContains, transform, "")
// {
//   return testing::ExplainMatchResult(
//     AllOf(testing::SizeIs(1),
//     )

//     transform_snapshot_matcher(arg.mutation().object().transforms_snapshot()), const T &value, MatchResultListener
//     *listener)
// }

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

TEST_F(ObjectSynchronizerTest, InitSucceeds) {
  // THEN the timer interface's setTimer function is called once with the expected timer period
  auto* timer_interface_ptr = mock_timer_interface.get();
  EXPECT_CALL(*timer_interface_ptr, setTimer(std::chrono::duration<double>{1.0}, _));

  // WHEN the ObjectSynchronizer is created
  createObjectSynchronizer();
}

TEST_F(ObjectSynchronizerTest, AddFrameAsWorldObject) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  auto* timer_interface_ptr = mock_timer_interface.get();
  ON_CALL(*timer_interface_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
    timer_interface_ptr->onSetTimer(cb);
  });

  // GIVEN the TF listener has info about two frames. One frame is an internal Spot frame, and the other frame is from a
  // different source THEN we make one request for known frame IDs
  constexpr auto kExternalFrameId = "some_external_frame";
  auto* tf_listener_interface_ptr = mock_tf_listener_interface.get();
  EXPECT_CALL(*tf_listener_interface_ptr, getAllFrameNames)
      .WillOnce(Return(std::vector<std::string>{"MyRobot/body", kExternalFrameId}));

  // GIVEN the TF listener always returns identity transforms
  // THEN we look up the transform from Spot's odom frame to the frame that was from a non-Spot source
  EXPECT_CALL(*tf_listener_interface_ptr, lookupTransform("MyRobot/odom", kExternalFrameId, _, _))
      .WillOnce(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  auto* world_object_client_interface_ptr = mock_world_object_client.get();

  // GIVEN Spot's list of world objects does not include any apriltags or other world objects
  // THEN we make two separate requests to list world objects: one to get info about apriltags, and the second to get
  // info about other world objects
  ::bosdyn::api::ListWorldObjectResponse list_apriltags_response;
  ::bosdyn::api::ListWorldObjectResponse list_objects_response;
  EXPECT_CALL(*world_object_client_interface_ptr, listWorldObjects)
      .WillOnce(Return(list_apriltags_response))
      .WillOnce(Return(list_objects_response));

  // THEN we send one MutateWorldObjectRequest
  // AND the request adds a new object
  // AND the new object's name matches the frame ID from the external source
  EXPECT_CALL(*world_object_client_interface_ptr,
              mutateWorldObject(AllOf(MutationAddsObject(), MutationTargetsObjectWhoseNameIs(kExternalFrameId))))
      .Times(1);

  // GIVEN the ObjectSynchronizer has been created
  createObjectSynchronizer();

  // WHEN the timer callback is triggered
  timer_interface_ptr->trigger();
}
}  // namespace spot_ros2::test
