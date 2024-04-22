// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/api/world_object.pb.h>
#include <gmock/gmock-actions.h>
#include <gmock/gmock-matchers.h>
#include <gmock/gmock-more-matchers.h>
#include <gmock/gmock-spec-builders.h>
#include <gmock/gmock.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <rcl/time.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/time.hpp>
#include <spot_driver/fake/fake_parameter_interface.hpp>
#include <spot_driver/mock/mock_clock_interface.hpp>
#include <spot_driver/mock/mock_logger_interface.hpp>
#include <spot_driver/mock/mock_node_interface.hpp>
#include <spot_driver/mock/mock_state_client.hpp>
#include <spot_driver/mock/mock_state_publisher_middleware_handle.hpp>
#include <spot_driver/mock/mock_tf_broadcaster_interface.hpp>
#include <spot_driver/mock/mock_tf_listener_interface.hpp>
#include <spot_driver/mock/mock_time_sync_api.hpp>
#include <spot_driver/mock/mock_timer_interface.hpp>
#include <spot_driver/mock/mock_world_object_client.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/robot_state_test_tools.hpp>
#include <spot_driver/serialization.hpp>
#include <spot_driver/types.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tl_expected/expected.hpp>
#include <utility>

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
using ::testing::IsEmpty;
using ::testing::Key;
using ::testing::Not;
using ::testing::Property;
using ::testing::Return;
using ::testing::SizeIs;
using ::testing::StrEq;
using ::testing::UnorderedElementsAre;
using ::testing::Unused;
using ::testing::Value;

MATCHER(MutationAddsObject, "") {
  return ExplainMatchResult(Eq(arg.mutation().action()),
                            ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_ADD,
                            result_listener);
}

MATCHER(ListObjectRequestIsForDrawableObjects, "") {
  return std::find(arg.object_type().cbegin(), arg.object_type().cend(),
                   ::bosdyn::api::WorldObjectType::WORLD_OBJECT_DRAWABLE) != arg.object_type().end();
}

MATCHER(MutationChangesObject, "") {
  return testing::ExplainMatchResult(
      testing::Eq(arg.mutation().action()),
      ::bosdyn::api::MutateWorldObjectRequest_Action::MutateWorldObjectRequest_Action_ACTION_CHANGE, result_listener);
}

MATCHER_P(MutationTargetsObjectWhoseNameIs, object_name, "") {
  return testing::ExplainMatchResult(StrEq(arg.mutation().object().name()), object_name, result_listener);
}

MATCHER_P(MutationTargetsObjectWhoseIdIs, object_id, "") {
  return testing::ExplainMatchResult(Eq(arg.mutation().object().id()), object_id, result_listener);
}

MATCHER_P2(MutationTransformChildAndParentFramesAre, child_frame_name, base_frame_name, "") {
  return testing::ExplainMatchResult(UnorderedElementsAre(Key(child_frame_name), Key(base_frame_name)),
                                     arg.mutation().object().transforms_snapshot().child_to_parent_edge_map(),
                                     result_listener);
}

MATCHER(MutationFrameTreeSnapshotIsValid, "") {
  return ::bosdyn::api::ValidateFrameTreeSnapshot(arg.mutation().object().transforms_snapshot()) ==
         ::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID;
}

inline const auto kMutateObjectResponseSuccess = [] {
  ::bosdyn::api::MutateWorldObjectResponse response;
  response.set_status(::bosdyn::api::MutateWorldObjectResponse_Status::MutateWorldObjectResponse_Status_STATUS_OK);
  return response;
}();

constexpr auto kExternalFrameId = "some_external_frame";
constexpr auto kErrorMessage = "some error message";
}  // namespace

namespace spot_ros2::test {
/**
 * @brief Wrapper class that exposes ObjectSynchronizer::addManagedFrame() as a public function to facilitate testing.
 * @details Since the set of managed frames is the point of interface between ObjectSynchronizer's two timer callback
 * functions, calling addManagedFrame independent of these functions allows the TF broadcaster functionality to be
 * tested in isolation from the world object monitoring functionality.
 */
class ObjectSynchronizerForTesting : public ObjectSynchronizer {
 public:
  ObjectSynchronizerForTesting(const std::shared_ptr<WorldObjectClientInterface>& world_object_client_interface,
                               const std::shared_ptr<TimeSyncApi>& time_sync_api,
                               std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                               std::unique_ptr<LoggerInterfaceBase> logger_interface,
                               std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                               std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                               std::unique_ptr<TimerInterfaceBase> world_object_update_timer,
                               std::unique_ptr<TimerInterfaceBase> tf_broadcaster_timer,
                               std::unique_ptr<ClockInterfaceBase> clock_interface)
      : ObjectSynchronizer{world_object_client_interface,
                           time_sync_api,
                           std::move(parameter_interface),
                           std::move(logger_interface),
                           std::move(tf_broadcaster_interface),
                           std::move(tf_listener_interface),
                           std::move(world_object_update_timer),
                           std::move(tf_broadcaster_timer),
                           std::move(clock_interface)} {}

  void addManagedFrame(const std::string& frame_id) { ObjectSynchronizer::addManagedFrame(frame_id); }
};

class ObjectSynchronizerTest : public ::testing::Test {
 public:
  void SetUp() override {
    mock_world_object_client = std::make_shared<MockWorldObjectClient>();
    mock_time_sync_api = std::make_shared<MockTimeSyncApi>();

    fake_parameter_interface = std::make_unique<FakeParameterInterface>();
    fake_parameter_interface->spot_name = "MyRobot";

    mock_logger_interface = std::make_unique<MockLoggerInterface>();
    mock_tf_broadcaster_interface = std::make_unique<MockTfBroadcasterInterface>();
    mock_tf_listener_interface = std::make_unique<MockTfListenerInterface>();
    mock_world_object_update_timer = std::make_unique<MockTimerInterface>();
    mock_tf_broadcaster_timer = std::make_unique<MockTimerInterface>();
    mock_clock_interface = std::make_unique<MockClockInterface>();

    mock_logger_interface_ptr = mock_logger_interface.get();
    mock_tf_broadcaster_interface_ptr = mock_tf_broadcaster_interface.get();
    mock_tf_listener_interface_ptr = mock_tf_listener_interface.get();
    mock_world_object_update_timer_ptr = mock_world_object_update_timer.get();
    mock_tf_broadcaster_timer_ptr = mock_tf_broadcaster_timer.get();
    mock_clock_interface_ptr = mock_clock_interface.get();
  }

  void createObjectSynchronizer() {
    object_synchronizer = std::make_unique<ObjectSynchronizerForTesting>(
        mock_world_object_client, mock_time_sync_api, std::move(fake_parameter_interface),
        std::move(mock_logger_interface), std::move(mock_tf_broadcaster_interface),
        std::move(mock_tf_listener_interface), std::move(mock_world_object_update_timer),
        std::move(mock_tf_broadcaster_timer), std::move(mock_clock_interface));
  }

  void registerTimerCallbacks() const {
    ON_CALL(*mock_world_object_update_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
      mock_world_object_update_timer_ptr->onSetTimer(cb);
    });

    ON_CALL(*mock_tf_broadcaster_timer_ptr, setTimer).WillByDefault([&](Unused, const std::function<void()>& cb) {
      mock_tf_broadcaster_timer_ptr->onSetTimer(cb);
    });
  }

  std::shared_ptr<MockWorldObjectClient> mock_world_object_client;
  std::shared_ptr<MockTimeSyncApi> mock_time_sync_api;

  std::unique_ptr<FakeParameterInterface> fake_parameter_interface;

  // Don't attempt to access these after createObjectSynchronizer() is called, since they get moved in that function
  std::unique_ptr<MockLoggerInterface> mock_logger_interface;
  std::unique_ptr<MockTfBroadcasterInterface> mock_tf_broadcaster_interface;
  std::unique_ptr<MockTfListenerInterface> mock_tf_listener_interface;
  std::unique_ptr<MockTimerInterface> mock_world_object_update_timer;
  std::unique_ptr<MockTimerInterface> mock_tf_broadcaster_timer;
  std::unique_ptr<MockClockInterface> mock_clock_interface;

  // Use these pointers to interact with the mocks during tests
  MockLoggerInterface* mock_logger_interface_ptr = nullptr;
  MockTfBroadcasterInterface* mock_tf_broadcaster_interface_ptr = nullptr;
  MockTfListenerInterface* mock_tf_listener_interface_ptr = nullptr;
  MockTimerInterface* mock_world_object_update_timer_ptr = nullptr;
  MockTimerInterface* mock_tf_broadcaster_timer_ptr = nullptr;
  MockClockInterface* mock_clock_interface_ptr = nullptr;

  std::unique_ptr<ObjectSynchronizerForTesting> object_synchronizer;
};

TEST_F(ObjectSynchronizerTest, InitSucceeds) {
  // THEN the timer interface's setTimer function is called once with the expected timer period
  EXPECT_CALL(*mock_world_object_update_timer_ptr, setTimer(std::chrono::duration<double>{1.0}, _));
  EXPECT_CALL(*mock_tf_broadcaster_timer, setTimer(std::chrono::duration<double>{0.1}, _));

  // WHEN the ObjectSynchronizer is created
  createObjectSynchronizer();
}

TEST_F(ObjectSynchronizerTest, AddFrameAsWorldObject) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  registerTimerCallbacks();

  ON_CALL(*mock_clock_interface_ptr, now).WillByDefault(Return(rclcpp::Time{0, 0, RCL_ROS_TIME}));

  // GIVEN the TF listener has info about two frames. One frame is an internal Spot frame, and the other frame is from a
  // different source.
  // THEN we make one request for known frame IDs
  EXPECT_CALL(*mock_tf_listener_interface_ptr, getAllFrameNames)
      .WillOnce(Return(std::vector<std::string>{"MyRobot/body", kExternalFrameId}));

  // GIVEN the TF listener always returns identity transforms
  // THEN we look up the transform from Spot's odom frame to the frame that was from a non-Spot source
  EXPECT_CALL(*mock_tf_listener_interface_ptr, lookupTransform("MyRobot/odom", kExternalFrameId, _))
      .WillOnce(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  // GIVEN Spot's list of world objects does not include any apriltags or other world objects
  // THEN we make two separate requests to list world objects: one to get info about immutable objects, the second to
  // get info about mutable objects
  ::bosdyn::api::ListWorldObjectResponse list_mutable_objects_response;
  ::bosdyn::api::ListWorldObjectResponse list_immutable_objects_response;
  EXPECT_CALL(*mock_world_object_client, listWorldObjects)
      .WillOnce(Return(list_immutable_objects_response))
      .WillOnce(Return(list_mutable_objects_response));

  // THEN we send one MutateWorldObjectRequest
  // AND the request adds a new object
  // AND the new object's name matches the frame ID from the external source
  // AND the new object's frame tree snapshot is valid
  // AND the new object's frame tree snapshot adds a transform from the base frame to the new frame ID
  EXPECT_CALL(*mock_world_object_client,
              mutateWorldObject(AllOf(MutationAddsObject(), MutationTargetsObjectWhoseNameIs(kExternalFrameId),
                                      MutationFrameTreeSnapshotIsValid(),
                                      MutationTransformChildAndParentFramesAre("odom", kExternalFrameId))))
      .WillOnce(
          Return(tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string>{kMutateObjectResponseSuccess}));

  EXPECT_CALL(*mock_logger_interface_ptr, logWarn).Times(0);
  EXPECT_CALL(*mock_logger_interface_ptr, logError).Times(0);

  // GIVEN the ObjectSynchronizer has been created
  // Note: for this test, this must only be called after registering all expected calls with the mocks
  createObjectSynchronizer();
  // GIVEN before the callback to sync world objects is triggered, the ObjectSynchronizer has no managed frames
  ASSERT_THAT(object_synchronizer->getManagedFrames(), IsEmpty());

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();

  // THEN the ObjectSynchronizer is managing a single frame matching the external frame ID which was published via TF.
  EXPECT_THAT(object_synchronizer->getManagedFrames(), AllOf(SizeIs(1), Contains(kExternalFrameId)));
}

TEST_F(ObjectSynchronizerTest, ModifyFrameForExistingWorldObject) {
  // GIVEN the timer interface's setTimer function registers the internal callback function to sync the world objects
  registerTimerCallbacks();

  ON_CALL(*mock_clock_interface_ptr, now).WillByDefault(Return(rclcpp::Time{0, 0, RCL_ROS_TIME}));

  // GIVEN the TF listener has info about two frames. One frame is an internal Spot frame, and the other frame is from a
  // different source.
  // THEN we make one request for known frame IDs
  constexpr int32_t kObjectId = 100;
  EXPECT_CALL(*mock_tf_listener_interface_ptr, getAllFrameNames)
      .WillOnce(Return(std::vector<std::string>{"MyRobot/body", kExternalFrameId}));

  // GIVEN the TF listener always returns identity transforms
  // THEN we look up the transform from Spot's odom frame to the frame that was from a non-Spot source
  EXPECT_CALL(*mock_tf_listener_interface_ptr, lookupTransform("MyRobot/odom", kExternalFrameId, _))
      .WillOnce(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  // GIVEN Spot's list of world objects includes a single existing mutable object
  // THEN we make two separate requests to list world objects: one to get info about immutable objects, the second to
  // get info about mutable objects
  ::bosdyn::api::ListWorldObjectResponse list_immutable_objects_response;
  ::bosdyn::api::ListWorldObjectResponse list_mutable_objects;
  auto* object = list_mutable_objects.add_world_objects();
  *object->mutable_name() = kExternalFrameId;
  object->set_id(kObjectId);
  addRootFrame(object->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at a nonzero pose relative to the odom frame
  addTransform(object->mutable_transforms_snapshot(), kExternalFrameId, "odom", 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0);
  EXPECT_CALL(*mock_world_object_client, listWorldObjects)
      .WillOnce(Return(list_immutable_objects_response))
      .WillOnce(Return(list_mutable_objects));

  // THEN we send one MutateWorldObjectRequest
  // AND the request modifies an existing object
  // AND the object's name matches the frame ID from the external source and the name of the existing object
  // AND the target object's ID matches the ID of the existing object
  // AND the mutation applies a valid frame tree snapshot
  // AND the mutation's frame tree snapshot adds a transform from the base frame to the new frame ID
  EXPECT_CALL(*mock_world_object_client,
              mutateWorldObject(AllOf(MutationChangesObject(), MutationTargetsObjectWhoseNameIs(kExternalFrameId),
                                      MutationTargetsObjectWhoseIdIs(kObjectId), MutationFrameTreeSnapshotIsValid(),
                                      MutationTransformChildAndParentFramesAre("odom", kExternalFrameId))))
      .WillOnce(
          Return(tl::expected<::bosdyn::api::MutateWorldObjectResponse, std::string>{kMutateObjectResponseSuccess}));

  // THEN no warning or error messages are logged
  EXPECT_CALL(*mock_logger_interface_ptr, logWarn).Times(0);
  EXPECT_CALL(*mock_logger_interface_ptr, logError).Times(0);

  // GIVEN the ObjectSynchronizer has been created
  // Note: for this test, this must only be called after registering all expected calls with the mocks
  createObjectSynchronizer();
  // GIVEN before the callback to sync world objects is triggered, the ObjectSynchronizer has no managed frames
  EXPECT_THAT(object_synchronizer->getManagedFrames(), IsEmpty());

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();

  // THEN the ObjectSynchronizer is managing the single non-Spot frame which the TF listener has info about
  EXPECT_THAT(object_synchronizer->getManagedFrames(), AllOf(SizeIs(1), Contains(kExternalFrameId)));
}

TEST_F(ObjectSynchronizerTest, TfLookupError) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  registerTimerCallbacks();

  ON_CALL(*mock_clock_interface_ptr, now).WillByDefault(Return(rclcpp::Time{0, 0, RCL_ROS_TIME}));

  // GIVEN the TF listener has info about one frame from a non-Spot source
  ON_CALL(*mock_tf_listener_interface_ptr, getAllFrameNames)
      .WillByDefault(Return(std::vector<std::string>{kExternalFrameId}));

  // GIVEN the TF listener will return a failure
  EXPECT_CALL(*mock_tf_listener_interface_ptr, lookupTransform)
      .WillRepeatedly(Return(tl::make_unexpected(kErrorMessage)));

  auto* world_object_client_interface_ptr = mock_world_object_client.get();

  // GIVEN Spot's list of world objects does not include any objects
  ::bosdyn::api::ListWorldObjectResponse list_objects_response;
  ON_CALL(*world_object_client_interface_ptr, listWorldObjects).WillByDefault(Return(list_objects_response));

  // THEN no request is made to mutate objects
  EXPECT_CALL(*world_object_client_interface_ptr, mutateWorldObject).Times(0);

  // THEN a warning message is reported that describes the TF lookup error
  EXPECT_CALL(*mock_logger_interface_ptr, logWarn(HasSubstr(kErrorMessage))).Times(1);

  // GIVEN the ObjectSynchronizer has been created
  createObjectSynchronizer();

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();

  // THEN the ObjectSynchronizer is not managing any frames after the update
  EXPECT_THAT(object_synchronizer->getManagedFrames(), IsEmpty());
}

TEST_F(ObjectSynchronizerTest, ListImmutableObjectsError) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  registerTimerCallbacks();

  ON_CALL(*mock_clock_interface_ptr, now).WillByDefault(Return(rclcpp::Time{0, 0, RCL_ROS_TIME}));

  // GIVEN the TF listener has info about one frame from a non-Spot source
  ON_CALL(*mock_tf_listener_interface_ptr, getAllFrameNames)
      .WillByDefault(Return(std::vector<std::string>{kExternalFrameId}));

  // GIVEN the TF listener will return identity transforms
  ON_CALL(*mock_tf_listener_interface_ptr, lookupTransform)
      .WillByDefault(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  // GIVEN requesting info about immutable world objects will fail
  EXPECT_CALL(*mock_world_object_client, listWorldObjects(Not(ListObjectRequestIsForDrawableObjects())))
      .WillOnce(Return(tl::make_unexpected(kErrorMessage)));

  // THEN no request is made to mutate objects
  EXPECT_CALL(*mock_world_object_client, mutateWorldObject).Times(0);

  // THEN an error message is reported that describes the failure
  EXPECT_CALL(*mock_logger_interface_ptr,
              logError(AllOf(HasSubstr("Failed to list non-mutable objects: "), HasSubstr(kErrorMessage))))
      .Times(1);

  // GIVEN the ObjectSynchronizer has been created
  // Note: for this test, this must only be called after registering all expected calls with the mocks
  createObjectSynchronizer();

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();

  // THEN the ObjectSynchronizer is not managing any frames
  EXPECT_THAT(object_synchronizer->getManagedFrames(), IsEmpty());
}

TEST_F(ObjectSynchronizerTest, ListMutableObjectsError) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  registerTimerCallbacks();

  ON_CALL(*mock_clock_interface_ptr, now).WillByDefault(Return(rclcpp::Time{0, 0, RCL_ROS_TIME}));

  // GIVEN the TF listener has info about one frame from a non-Spot source
  ON_CALL(*mock_tf_listener_interface_ptr, getAllFrameNames)
      .WillByDefault(Return(std::vector<std::string>{kExternalFrameId}));

  // GIVEN the TF listener will return identity transforms
  ON_CALL(*mock_tf_listener_interface_ptr, lookupTransform)
      .WillByDefault(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  // GIVEN requesting info about immutable world objects will succeed
  // AND requesting info about mutable world objects will fail
  // THEN we first reuqest info about all objects that are not drawables, then request info about drawables
  {
    testing::InSequence seq;
    ::bosdyn::api::ListWorldObjectResponse list_objects_response;
    EXPECT_CALL(*mock_world_object_client, listWorldObjects(Not(ListObjectRequestIsForDrawableObjects())))
        .WillOnce(Return(list_objects_response));
    EXPECT_CALL(*mock_world_object_client, listWorldObjects(ListObjectRequestIsForDrawableObjects()))
        .WillOnce(Return(tl::make_unexpected(kErrorMessage)));
  }

  // THEN no request is made to mutate objects
  EXPECT_CALL(*mock_world_object_client, mutateWorldObject).Times(0);

  // THEN an error message is reported that describes the failure
  EXPECT_CALL(*mock_logger_interface_ptr,
              logError(AllOf(HasSubstr("Failed to list mutable objects: "), HasSubstr(kErrorMessage))))
      .Times(1);

  // GIVEN the ObjectSynchronizer has been created
  createObjectSynchronizer();

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();

  // THEN the ObjectSynchronizer is not managing any frames
  EXPECT_THAT(object_synchronizer->getManagedFrames(), IsEmpty());
}

TEST_F(ObjectSynchronizerTest, MutateObjectsError) {
  // GIVEN the timer interface's setTimer function registers the internal callback function
  registerTimerCallbacks();

  ON_CALL(*mock_clock_interface_ptr, now).WillByDefault(Return(rclcpp::Time{0, 0, RCL_ROS_TIME}));

  // GIVEN the TF listener has info about one frame from a non-Spot source
  ON_CALL(*mock_tf_listener_interface_ptr, getAllFrameNames)
      .WillByDefault(Return(std::vector<std::string>{kExternalFrameId}));

  // GIVEN the TF listener will return identity transforms
  ON_CALL(*mock_tf_listener_interface_ptr, lookupTransform)
      .WillByDefault(Return(
          tl::expected<geometry_msgs::msg::TransformStamped, std::string>{geometry_msgs::msg::TransformStamped{}}));

  // GIVEN requesting info about world objects always succeeds
  ::bosdyn::api::ListWorldObjectResponse list_objects_response;
  ON_CALL(*mock_world_object_client, listWorldObjects).WillByDefault(Return(list_objects_response));

  // GIVEN mutating world objects will always fail
  ON_CALL(*mock_world_object_client, mutateWorldObject).WillByDefault(Return(tl::make_unexpected(kErrorMessage)));

  // THEN a warning message is reported that describes the error
  EXPECT_CALL(*mock_logger_interface_ptr,
              logWarn(AllOf(HasSubstr("Failed to modify world object: "), HasSubstr(kErrorMessage))))
      .Times(1);

  // GIVEN the ObjectSynchronizer has been created
  // Note: for this test, this must only be called after registering all expected calls with the mocks
  createObjectSynchronizer();

  // WHEN the timer callback is triggered
  mock_world_object_update_timer_ptr->trigger();

  // THEN the ObjectSynchronizer is not managing any frames
  EXPECT_THAT(object_synchronizer->getManagedFrames(), IsEmpty());
}

TEST_F(ObjectSynchronizerTest, PublishWorldObjectTransforms) {
  // GIVEN the callback to broadcast TF data has been registered with the appropriate timer
  registerTimerCallbacks();

  // GIVEN Spot's WorldObject API will report two objects
  // THEN we request info about world objects from Spot's WorldObject API
  ::bosdyn::api::ListWorldObjectResponse list_objects_response;
  auto* object_dock = list_objects_response.add_world_objects();
  *object_dock->mutable_name() = "dock";
  object_dock->set_id(99);
  object_dock->mutable_dock_properties()->set_dock_id(100);
  addRootFrame(object_dock->mutable_transforms_snapshot(), "odom");
  addTransform(object_dock->mutable_transforms_snapshot(), "dock", "odom", 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

  auto* object_drawable = list_objects_response.add_world_objects();
  *object_drawable->mutable_name() = "my_object";
  object_drawable->set_id(100);
  auto* props = object_drawable->mutable_drawable_properties()->Add();
  *props->mutable_label() = "TEXT";
  addRootFrame(object_drawable->mutable_transforms_snapshot(), "odom");
  addTransform(object_drawable->mutable_transforms_snapshot(), "my_object", "odom", 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  auto* world_object_client_interface_ptr = mock_world_object_client.get();
  EXPECT_CALL(*world_object_client_interface_ptr, listWorldObjects).WillOnce(Return(list_objects_response));

  // THEN we broadcast a single transform, where the child frame ID of this transform is the name of the object whose TF
  // frame is not being managed by the ObjectSynchronizer with the robot name applied as a namespace
  EXPECT_CALL(*mock_tf_broadcaster_interface_ptr,
              sendDynamicTransforms(AllOf(
                  SizeIs(1), Contains(Field("child_frame_id", &geometry_msgs::msg::TransformStamped::child_frame_id,
                                            StrEq("MyRobot/dock"))))))
      .Times(1);

  // THEN no warning or error messages are logged
  EXPECT_CALL(*mock_logger_interface_ptr, logWarn).Times(0);
  EXPECT_CALL(*mock_logger_interface_ptr, logError).Times(0);

  // GIVEN the ObjectSynchronizer has been created
  createObjectSynchronizer();

  // GIVEN the ObjectSynchronizer is managing one frame that corresponds to one of the reported world objects
  object_synchronizer->addManagedFrame("my_object");
  ASSERT_THAT(object_synchronizer->getManagedFrames(), Contains("my_object"));

  // WHEN the timer callback to broadcast TF data is triggered
  mock_tf_broadcaster_timer_ptr->trigger();
}
}  // namespace spot_ros2::test
