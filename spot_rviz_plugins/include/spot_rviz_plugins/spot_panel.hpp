#ifndef SPOT_RVIZ_PLUGINS__SPOT_PANEL_HPP_
#define SPOT_RVIZ_PLUGINS__SPOT_PANEL_HPP_

#include <QBasicTimer>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/lease_array.hpp>
#include <spot_msgs/msg/mobility_params.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/srv/dock.hpp>
#include <spot_msgs/srv/set_locomotion.hpp>
#include <spot_msgs/srv/set_velocity.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace spot_rviz_plugins {

class SpotPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit SpotPanel(QWidget* parent = 0);
  virtual void save(rviz_common::Config config) const;
  virtual void load(const rviz_common::Config& config);

 private Q_SLOTS:  // NOLINT
  void sit();
  void stand();
  void claimLease();
  void releaseLease();
  void powerOn();
  void powerOff();
  void setMaxVel();
  void gentleStop();
  void hardStop();
  void releaseStop();
  void stop();
  void selfRight();
  void dock();
  void undock();
  void setRobotName();

 private:
  void setupStopButtons();
  void setupSpinBoxes();
  void setControlButtons();
  void initialiseRosComponents();
  void destroyRosComponents();
  void callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service);
  template <typename C>
  void callCustomTriggerService(typename rclcpp::Client<C>::SharedPtr service,
                                const typename C::Request& serviceRequest);
  void updateLabelTextWithLimit(QLabel* label, double limit_lower, double limit_upper);
  void leaseCallback(const spot_msgs::msg::LeaseArray::ConstSharedPtr& leases);
  void estopCallback(const spot_msgs::msg::EStopStateArray::ConstSharedPtr& estops);
  void mobilityParamsCallback(const spot_msgs::msg::MobilityParams::ConstSharedPtr& params);
  void batteryCallback(const spot_msgs::msg::BatteryStateArray::ConstSharedPtr& battery);
  void powerCallback(const spot_msgs::msg::PowerState::ConstSharedPtr& power);
  template <typename C>
  void serviceResponseCallback(std::string serviceName, typename rclcpp::Client<C>::SharedFuture future);

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sitService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr standService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr claimLeaseService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr releaseLeaseService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr powerOnService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr powerOffService_;
  rclcpp::Client<spot_msgs::srv::SetVelocity>::SharedPtr maxVelocityService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hardStopService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gentleStopService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr releaseStopService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stopService_;
  rclcpp::Client<spot_msgs::srv::Dock>::SharedPtr dockService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undockService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr selfRightService_;

  rclcpp::Subscription<spot_msgs::msg::LeaseArray>::SharedPtr leaseSub_;
  rclcpp::Subscription<spot_msgs::msg::EStopStateArray>::SharedPtr estopSub_;
  rclcpp::Subscription<spot_msgs::msg::MobilityParams>::SharedPtr mobilityParamsSub_;
  rclcpp::Subscription<spot_msgs::msg::BatteryStateArray>::SharedPtr batterySub_;
  rclcpp::Subscription<spot_msgs::msg::PowerState>::SharedPtr powerSub_;

  QPushButton* claimLeaseButton;
  QPushButton* releaseLeaseButton;
  QPushButton* powerOnButton;
  QPushButton* powerOffButton;
  QPushButton* sitButton;
  QPushButton* standButton;
  QPushButton* setMaxVelButton;
  QPushButton* hardStopButton;
  QPushButton* gentleStopButton;
  QPushButton* releaseStopButton;
  QPushButton* stopButton;
  QPushButton* dockButton;
  QPushButton* undockButton;
  QPushButton* selfRightButton;
  QPushButton* setRobotNameButton;

  QLabel* linearXLabel;
  QLabel* linearYLabel;
  QLabel* angularZLabel;
  QLabel* statusLabel;
  QLabel* estimatedRuntimeLabel;
  QLabel* batteryStateLabel;
  QLabel* motorStateLabel;
  QLabel* batteryTempLabel;
  QLabel* estopLabel;

  QLineEdit* robotNameLineEdit;

  QSpinBox* dockFiducialSpin;

  QDoubleSpinBox* linearXSpin;
  QDoubleSpinBox* linearYSpin;
  QDoubleSpinBox* angularZSpin;

  // Qt timer we use to ensure that the node in this panel is spun regularly
  QBasicTimer spin_timer;
  // Once the timer starts this function is called every time its duration elapses
  void timerEvent(QTimerEvent* event) override;

  spot_msgs::msg::MobilityParams _lastMobilityParams;

  bool haveLease;
  bool isEStopped;
  bool motionAllowed;
};

}  // end namespace spot_rviz_plugins

#endif  // SPOT_RVIZ_PLUGINS__SPOT_PANEL_HPP_
