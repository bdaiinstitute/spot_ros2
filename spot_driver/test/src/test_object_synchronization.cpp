// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/geometry.pb.h>
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
#include <spot_driver/mock/mock_world_object_client.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/robot_state_test_tools.hpp>
#include <spot_driver/types.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tl_expected/expected.hpp>
#include <utility>
#include "spot_driver/mock/mock_clock_interface.hpp"

namespace {
using ::testing::_;
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::Eq;
using ::testing::ExplainMatchResult;
using ::testing::Field;
using ::testing::HasSubstr;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Key;
using ::testing::Property;
using ::testing::Return;
using ::testing::StrEq;
using ::testing::UnorderedElementsAre;
using ::testing::Unused;
using ::testing::Value;

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

MATCHER_P2(MutationTransformChildAndParentFramesAre, child_frame_name, base_frame_name, "") {
  return testing::ExplainMatchResult(UnorderedElementsAre(Key(child_frame_name), Key(base_frame_name)),
                                     arg.mutation().object().transforms_snapshot().child_to_parent_edge_map(),
                                     result_listener);
}

MATCHER(MutationFrameTreeSnapshotIsValid, "") {
  return ::bosdyn::api::ValidateFrameTreeSnapshot(arg.mutation().object().transforms_snapshot()) ==
         ::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID;
  // return
  // testing::ExplainMatchResult(Value(::bosdyn::api::ValidateFrameTreeSnapshot(arg.mutation().object().transforms_snapshot()),
  // Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID)), result_listener);
}

// MATCHER_P(SE3PoseEq, pose, "")
// {
//   ::bosdyn::api::SE3Pose p;
//   AllOf(Field("position", &::bosdyn::api::SE3Pose::position, Eq(pose.position())),
//   Field("rotation", &::bosdyn::api::SE3Pose::rotation, Eq(pose.position())));

//   return testing::ExplainMatchResult(Eq(arg), transform, result_listener);
// }

// MATCHER_P(MutationObjectTransformSnapshotContains, transform, "")
// {
//   return testing::ExplainMatchResult(Field(&::bosdyn::api::MutateWorldObjectRequest::mutation,
//     Field(&::bosdyn::api::WorldObject::transforms_snapshot,
//       Field(&::bosdyn::api::FrameTreeSnapshot::child_to_parent_edge_map, Contains(transform)))), arg,
//       result_listener);
// }

constexpr auto kErrorMessage = "Some error message.";
}  // namespace

namespace spot_ros2::test {
class ObjectSynchronizerTest : public ::testing::Test {
 public:
  void SetUp() override {
    mock_world_object_client = std::make_shared<MockWorldObjectClient>();
    mock_time_sync_api = std::make_shared<MockTimeSyncApi>();

    fake_parameter_interface = std::make_unique<FakeParameterInterface>();
    fake_parameter_interface->spot_name = "MyRobot";

    mock_logger_interface = std::make_unique<MockLoggerInterface>();
    mock_tf_broadcaster_interface = std::make_unique<MockTfInterface>();
    mock_tf_listener_interface = std::make_unique<MockTfListenerInterface>();
    mock_world_object_update_timer = std::make_unique<MockTimerInterface>();
    mock_tf_broadcaster_timer = std::make_unique<MockTimerInterface>();
    mock_clock_interface = std::make_unique<MockClockInterface>();
  }

  void createObjectSynchronizer() {
    object_synchronizer = std::make_unique<ObjectSynchronizer>(
        mock_world_object_client, mock_time_sync_api, std::move(fake_parameter_interface),
        std::move(mock_logger_interface), std::move(mock_tf_broadcaster_interface),
        std::move(mock_tf_listener_interface), std::move(mock_world_object_update_timer),
        std::move(mock_tf_broadcaster_timer), std::move(mock_clock_interface));
  }

  void setInternalTimerCallback() const {
    auto* mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
    ON_CALL(*mock_world_object_update_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
      mock_world_object_update_timer_ptr->onSetTimer(cb);
    });

    auto* mock_tf_broadcaster_timer_ptr = mock_tf_broadcaster_timer.get();
    ON_CALL(*mock_tf_broadcaster_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
      mock_tf_broadcaster_timer_ptr->onSetTimer(cb);
    });
  }
  std::shared_ptr<MockWorldObjectClient> mock_world_object_client;
  std::shared_ptr<MockTimeSyncApi> mock_time_sync_api;

  std::unique_ptr<FakeParameterInterface> fake_parameter_interface;

  std::unique_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<MockTfInterface> mock_tf_broadcaster_interface;
  std::unique_ptr<MockTfListenerInterface> mock_tf_listener_interface;
  std::unique_ptr<MockTimerInterface> mock_world_object_update_timer;
  std::unique_ptr<MockTimerInterface> mock_tf_broadcaster_timer;
  std::unique_ptr<MockClockInterface> mock_clock_interface;

  std::unique_ptr<ObjectSynchronizer> object_synchronizer;
};

TEST_F(ObjectSynchronizerTest, InitSucceeds) {
  // THEN the timer interface's setTimer function is called once with the expected timer period
  auto* mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
  EXPECT_CALL(*mock_world_object_update_timer_ptr, setTimer(std::chrono::duration<double>{1.0}, _));
  auto* mock_tf_update_timer_ptr = mock_tf_broadcaster_timer.get();
  EXPECT_CALL(*mock_tf_update_timer_ptr, setTimer(std::chrono::duration<double>{0.1}, _));

  // WHEN the ObjectSynchronizer is created
  createObjectSynchronizer();
}

TEST_F(ObjectSynchronizerTest, AddFrameAsWorldObject) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  auto* mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
  ON_CALL(*mock_world_object_update_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
    mock_world_object_update_timer_ptr->onSetTimer(cb);
  });

  auto* mock_tf_broadcaster_timer_ptr = mock_tf_broadcaster_timer.get();
  ON_CALL(*mock_tf_broadcaster_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
    mock_tf_broadcaster_timer_ptr->onSetTimer(cb);
  });
  // auto* mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
  // setInternalTimerCallback();

  // GIVEN the TF listener has info about two frames. One frame is an internal Spot frame, and the other frame is from a
  // different source.
  // THEN we make one request for known frame IDs
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

  /*
  OTHER IMPORTANT THINGS TO TEST:
  - the object is of type DrawableFrame
  - if modifying an existing object, the request contains the ID of the object
  - we can handle the TF listener failing to look up a transform
  - we can handle the WorldObject client returning one of several possible failure cases (also make these more
  human-readable)
  */

  // THEN we send one MutateWorldObjectRequest
  // AND the request adds a new object
  // AND the new object's name matches the frame ID from the external source
  // AND the new object's frame tree snapshot is valid
  // AND the new object's frame tree snapshot adds a transform from the base frame to the new frame ID
  EXPECT_CALL(*world_object_client_interface_ptr,
              mutateWorldObject(AllOf(MutationAddsObject(), MutationTargetsObjectWhoseNameIs(kExternalFrameId),
                                      MutationFrameTreeSnapshotIsValid(),
                                      MutationTransformChildAndParentFramesAre("odom", kExternalFrameId))))
      .Times(1);

  // GIVEN the ObjectSynchronizer has been created
  createObjectSynchronizer();

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();
}

TEST_F(ObjectSynchronizerTest, ModifyFrameForExistingWorldObject) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  // auto* timer_interface_ptr = mock_world_object_update_timer.get();
  // ON_CALL(*timer_interface_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
  //   timer_interface_ptr->onSetTimer(cb);
  // });

  auto* mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
  ON_CALL(*mock_world_object_update_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
    mock_world_object_update_timer_ptr->onSetTimer(cb);
  });

  auto* mock_tf_broadcaster_timer_ptr = mock_tf_broadcaster_timer.get();
  ON_CALL(*mock_tf_broadcaster_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
    mock_tf_broadcaster_timer_ptr->onSetTimer(cb);
  });

  // auto* mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
  // setInternalTimerCallback();

  // GIVEN the TF listener has info about two frames. One frame is an internal Spot frame, and the other frame is from a
  // different source.
  // THEN we make one request for known frame IDs
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
  auto* object = list_objects_response.add_world_objects();
  *object->mutable_name() = kExternalFrameId;
  addRootFrame(object->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at a nonzero pose relative to the odom frame
  addTransform(object->mutable_transforms_snapshot(), kExternalFrameId, "odom", 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0);
  EXPECT_CALL(*world_object_client_interface_ptr, listWorldObjects)
      .WillOnce(Return(list_apriltags_response))
      .WillOnce(Return(list_objects_response))
      .WillRepeatedly(Return(list_objects_response));

  // THEN we send one MutateWorldObjectRequest
  // AND the request adds a new object
  // AND the new object's name matches the frame ID from the external source
  // AND the new object's frame tree snapshot is valid
  // AND the new object's frame tree snapshot adds a transform from the base frame to the new frame ID
  ::bosdyn::api::SE3Pose pose;
  pose.mutable_position()->set_x(1.0);
  pose.mutable_position()->set_y(2.0);
  pose.mutable_position()->set_z(3.0);
  pose.mutable_rotation()->set_w(1.0);
  pose.mutable_rotation()->set_x(0.0);
  pose.mutable_rotation()->set_y(0.0);
  pose.mutable_rotation()->set_z(0.0);
  EXPECT_CALL(*world_object_client_interface_ptr,
              mutateWorldObject(AllOf(MutationChangesObject(), MutationTargetsObjectWhoseNameIs(kExternalFrameId),
                                      // MutationObjectTransformSnapshotContains(pose)
                                      MutationFrameTreeSnapshotIsValid(),
                                      MutationTransformChildAndParentFramesAre("odom", kExternalFrameId))))
      .Times(1);

  // auto* tf_broadcaster_ptr = mock_tf_broadcaster_interface.get();
  // EXPECT_CALL(*tf_broadcaster_ptr, sendDynamicTransforms).Times(0);

  // GIVEN the ObjectSynchronizer has been created
  createObjectSynchronizer();

  // WHEN the timer callbacks are triggered
  mock_world_object_update_timer_ptr->trigger();
  mock_tf_broadcaster_timer_ptr->trigger();
}
}  // namespace spot_ros2::test
