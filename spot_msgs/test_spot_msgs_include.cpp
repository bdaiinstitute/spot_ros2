#include <gmock/gmock.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>

#include "spot_msgs/msg/battery_state.hpp"


// copied from r^3/test/utils
namespace rclcpp_testing {

    // Convenient test fixture to setup and teardown rclcpp.
    class Test : public ::testing::Test {
     public:
      void SetUp() override { rclcpp::init(0, nullptr); }
    
      void TearDown() override { rclcpp::shutdown(); }
    };
    
    // Convenient mock that can be used to set call expectations
    // on a subscriber callback.
    //
    // Usage example:
    //
    //     MessageType message;
    //     message.attribute = ...;
    //
    //     auto subscriber = rclcpp_testing::MockSubscriber<MessageType>{};
    //     EXPECT_CALL(subscriber, Call(Pointee(Eq(message))));
    //
    //     auto subscription = subscriber.create_subscription(node, topic, qos_history_depth);
    //
    template <typename MessageType>
    class MockSubscriber {
     public:
      MOCK_METHOD(void, Call, (const std::shared_ptr<MessageType>), (const));
    
      auto create_subscription(
          std::shared_ptr<rclcpp::Node> node,
          const std::string& topic,
          std::size_t qos_history_depth) const {
        return node->create_subscription<MessageType>(
          topic,
          qos_history_depth,
          [this](std::shared_ptr<MessageType> message) { Call(message); });
      }
    };
    
}  // namespace rclcpp_testing


namespace {

using ::testing::_;
using ::testing::Pointee;
using ::testing::Eq;

class CustomMsgsTest : public rclcpp_testing::Test {};

TEST_F(CustomMsgsTest, RosPackagesExist) {
  ASSERT_NO_THROW(ament_index_cpp::get_package_prefix("spot_msgs"));
}

TEST_F(CustomMsgsTest, PubSub) {
  constexpr auto qos_history_depth = 10;
  constexpr auto topic = "test_battery_state";

  auto node = std::make_shared<rclcpp::Node>("spot_msgs_test_node");
  auto publisher = node->create_publisher<spot_msgs::msg::BatteryState>(topic, qos_history_depth);

  spot_msgs::msg::BatteryState message;
  message.header.frame_id = "test_frame";

  auto subscriber = rclcpp_testing::MockSubscriber<spot_msgs::msg::BatteryState>{};
  EXPECT_CALL(subscriber, Call(Pointee(Eq(message))));

  [[maybe_unused]] auto subscription = subscriber.create_subscription(node, topic, qos_history_depth);
  publisher->publish(message);

  rclcpp::spin_some(node);
}

}  // namespace