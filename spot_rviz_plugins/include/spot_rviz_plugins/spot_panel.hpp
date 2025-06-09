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
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/mobility_params.hpp>
#include <spot_msgs/srv/dock.hpp>
#include <spot_msgs/srv/set_velocity.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
// #include <spot_msgs/TerrainParams.h>
// #include <spot_msgs/SetSwingHeight.h>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/lease_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/srv/set_locomotion.hpp>
/* #include <spot_cam/PTZDescriptionArray.h>
#include <spot_cam/StringMultiArray.h> */

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
  void sendBodyPose();
  void sendNeutralBodyPose();
  void setMaxVel();
  void gentleStop();
  void hardStop();
  void releaseStop();
  void stop();
  void setGait();
  void setSwingHeight();
  void setTerrainParams();
  void setObstacleParams();
  void allowMotion();
  void rollOverRight();
  void rollOverLeft();
  void selfRight();
  void dock();
  void undock();
  void setCamPTZ();
  void setCamScreen();
  void setCamLED(double);
  void camLookAtPoint();
  void camTrackPoint();

 private:
  // These maps allow us to set up the comboboxes for selections in the order
  // of the enum, and ensure that the correct value is sent when the user wants to set it
  // We can also use them to ensure that non-consecutive values are also correctly handled
  /*     const std::map<uint, std::string> gaitMap = {
          {spot_msgs::SetLocomotion::Request::HINT_UNKNOWN, "Unknown"},
          {spot_msgs::SetLocomotion::Request::HINT_TROT, "Trot"},
          {spot_msgs::SetLocomotion::Request::HINT_SPEED_SELECT_TROT, "Speed sel trot"},
          {spot_msgs::SetLocomotion::Request::HINT_CRAWL, "Crawl"},
          {spot_msgs::SetLocomotion::Request::HINT_AMBLE, "Amble"},
          {spot_msgs::SetLocomotion::Request::HINT_SPEED_SELECT_AMBLE, "Speed sel amble"},
          {spot_msgs::SetLocomotion::Request::HINT_AUTO,  "Auto"},
          {spot_msgs::SetLocomotion::Request::HINT_JOG, "Jog"},
          {spot_msgs::SetLocomotion::Request::HINT_HOP, "Hop"},
          {spot_msgs::SetLocomotion::Request::HINT_SPEED_SELECT_CRAWL, "Speed sel crawl"}
      };

      const std::map<uint, std::string> swingHeightMap = {
          {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_UNKNOWN, "Unknown"},
          {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_LOW, "Low"},
          {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_MEDIUM, "Medium"},
          {spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_HIGH, "High"}
      };

      const std::map<uint, std::string> gratedSurfacesMap = {
          {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_UNKNOWN, "Unknown"},
          {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_OFF, "Off"},
          {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_ON, "On"},
          {spot_msgs::TerrainParams::GRATED_SURFACES_MODE_AUTO, "Auto"}
      }; */

  void setupComboBoxes();
  void setupStopButtons();
  void setupSpinBoxes();
  void setControlButtons();
  void toggleBodyPoseButtons();
  void callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service);
  template <typename C, typename T>
  void callCustomTriggerService(typename rclcpp::Client<C>::SharedPtr service, T serviceRequest);
  void updateLabelTextWithLimit(QLabel* label, double limit_lower, double limit_upper);
  void leaseCallback(const spot_msgs::msg::LeaseArray::ConstSharedPtr& leases);
  void estopCallback(const spot_msgs::msg::EStopStateArray::ConstSharedPtr& estops);
  void mobilityParamsCallback(const spot_msgs::msg::MobilityParams::ConstSharedPtr& params);
  void batteryCallback(const spot_msgs::msg::BatteryStateArray::ConstSharedPtr& battery);
  void powerCallback(const spot_msgs::msg::PowerState::ConstSharedPtr& power);
  void motionAllowedCallback(const std_msgs::msg::Bool& motion_allowed);
  template <typename C>
  void serviceResponseCallback(std::string serviceName, typename rclcpp::Client<C>::SharedFuture future);
  //   void ptzCallback(const spot_cam::PTZDescriptionArray &ptz_descriptions);
  //   void screensCallback(const spot_cam::StringMultiArray &screens);
  //   void lookAtPoint(const bool track);

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
  // rclcpp::Client<> gaitService_;
  // rclcpp::Client<> swingHeightService_;
  // rclcpp::Client<> terrainParamsService_;
  // rclcpp::Client<> obstacleParamsService_;
  // rclcpp::Client<> allowMotionService_;
  // rclcpp::Client<> bodyPoseService_;
  rclcpp::Client<spot_msgs::srv::Dock>::SharedPtr dockService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undockService_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr selfRightService_;
  // rclcpp::Client<> rollOverLeftService_;
  // rclcpp::Client<> rollOverRightService_;

  rclcpp::Subscription<spot_msgs::msg::LeaseArray>::SharedPtr leaseSub_;
  rclcpp::Subscription<spot_msgs::msg::EStopStateArray>::SharedPtr estopSub_;
  rclcpp::Subscription<spot_msgs::msg::MobilityParams>::SharedPtr mobilityParamsSub_;
  rclcpp::Subscription<spot_msgs::msg::BatteryStateArray>::SharedPtr batterySub_;
  rclcpp::Subscription<spot_msgs::msg::PowerState>::SharedPtr powerSub_;
  // rclcpp::Subscription motionAllowedSub_;
  // rclcpp::Subscription camScreensSub_;
  // rclcpp::Subscription camPTZSub_;

  QPushButton* claimLeaseButton;
  QPushButton* releaseLeaseButton;
  QPushButton* powerOnButton;
  QPushButton* powerOffButton;
  QPushButton* setBodyPoseButton;
  QPushButton* setBodyNeutralButton;
  QPushButton* sitButton;
  QPushButton* standButton;
  QPushButton* setMaxVelButton;
  QPushButton* hardStopButton;
  QPushButton* gentleStopButton;
  QPushButton* releaseStopButton;
  QPushButton* stopButton;
  QPushButton* setGaitButton;
  QPushButton* setSwingHeightButton;
  QPushButton* setObstaclePaddingButton;
  QPushButton* setGratedSurfacesButton;
  QPushButton* setFrictionButton;
  QPushButton* allowMotionButton;
  QPushButton* dockButton;
  QPushButton* undockButton;
  QPushButton* selfRightButton;
  QPushButton* rollOverLeftButton;
  QPushButton* rollOverRightButton;

  QLabel* linearXLabel;
  QLabel* linearYLabel;
  QLabel* angularZLabel;
  QLabel* statusLabel;
  QLabel* bodyHeightLabel;
  QLabel* rollLabel;
  QLabel* pitchLabel;
  QLabel* yawLabel;
  QLabel* estimatedRuntimeLabel;
  QLabel* batteryStateLabel;
  QLabel* motorStateLabel;
  QLabel* batteryTempLabel;
  QLabel* estopLabel;

  QComboBox* gaitComboBox;
  QComboBox* swingHeightComboBox;
  QComboBox* gratedSurfacesComboBox;

  QSpinBox* dockFiducialSpin;

  QDoubleSpinBox* linearXSpin;
  QDoubleSpinBox* linearYSpin;
  QDoubleSpinBox* angularZSpin;
  QDoubleSpinBox* bodyHeightSpin;
  QDoubleSpinBox* rollSpin;
  QDoubleSpinBox* pitchSpin;
  QDoubleSpinBox* yawSpin;
  QDoubleSpinBox* frictionSpin;
  QDoubleSpinBox* obstaclePaddingSpin;

  // Qt timer we use to ensure that the node in this panel is spun regularly
  QBasicTimer spin_timer;
  // Once the timer starts this function is called every time its duration elapses
  void timerEvent(QTimerEvent* event) override;

  // Spot cam
  /*     ros::Publisher camLEDPub_;
      rclcpp::Client<> camSetPTZService_;
      rclcpp::Client<> camSetScreenService_;
      rclcpp::Client<> camLookAtPointService_;

      QPushButton* setPTZButton;
      QPushButton* setScreenButton;
      QComboBox* chooseScreenComboBox;
      QComboBox* choosePTZComboBox;
      QDoubleSpinBox* LEDSpinBox;
      QDoubleSpinBox* panSpinBox;
      QDoubleSpinBox* tiltSpinBox;
      QDoubleSpinBox* zoomSpinBox;

      QLineEdit* lookAtFrameLineEdit;
      QDoubleSpinBox* lookXSpinBox;
      QDoubleSpinBox* lookYSpinBox;
      QDoubleSpinBox* lookZSpinBox;
      QDoubleSpinBox* imageWidthSpinBox;
      QDoubleSpinBox* lookZoomSpinBox;
      QPushButton* lookAtPointButton;
      QPushButton* trackPointButton; */

  spot_msgs::msg::MobilityParams _lastMobilityParams;

  bool haveLease;
  bool isEStopped;
  bool motionAllowed;
};

}  // end namespace spot_rviz_plugins

#endif  // SPOT_RVIZ_PLUGINS__SPOT_PANEL_HPP_
