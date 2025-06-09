#include "spot_rviz_plugins/spot_panel.hpp"

#include <QDoubleValidator>
#include <QFile>
#include <QStandardItemModel>
#include <QUiLoader>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
// #include <tf/transform_datatypes.h>
#include <spot_msgs/msg/e_stop_state.hpp>
#include <std_msgs/msg/float32.hpp>
// #include <spot_msgs/SetObstacleParams.h>
// #include <spot_msgs/SetTerrainParams.h>
// #include <spot_msgs/PosedStand.h>
// #include <spot_cam/SetPTZState.h>
// #include <spot_cam/SetString.h>
// #include <spot_cam/LookAtPoint.h>
#include <string.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace spot_rviz_plugins {

SpotPanel::SpotPanel(QWidget* parent) {
  auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", "__node:=spot_driver_rviz_panel", "--"});
  client_node_ = rclcpp::Node::make_shared("_", "spot_driver_rviz_panel", options);

  std::string packagePath =
      ament_index_cpp::get_package_share_directory("spot_rviz_plugins") + "/resource/spot_panel.ui";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting ui file from package path %s", packagePath.c_str());
  QFile file(packagePath.c_str());
  file.open(QIODevice::ReadOnly);

  QUiLoader loader;
  QWidget* ui = loader.load(&file, parent);
  file.close();
  QVBoxLayout* topLayout = new QVBoxLayout();
  this->setLayout(topLayout);
  topLayout->addWidget(ui);
  haveLease = false;
  motionAllowed = false;

  sitService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/sit");
  standService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/stand");
  claimLeaseService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/claim");
  releaseLeaseService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/release");
  powerOnService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/power_on");
  powerOffService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/power_off");
  maxVelocityService_ = client_node_->create_client<spot_msgs::srv::SetVelocity>("/spot/velocity_limit");
  hardStopService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/estop/hard");
  gentleStopService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/estop/gentle");
  releaseStopService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/estop/release");
  stopService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/stop");
  // gaitService_ = nh_.serviceClient<spot_msgs::SetLocomotion>("/spot/locomotion_mode");
  // swingHeightService_ = nh_.serviceClient<spot_msgs::SetSwingHeight>("/spot/swing_height");
  // terrainParamsService_ = nh_.serviceClient<spot_msgs::SetTerrainParams>("/spot/terrain_params");
  // obstacleParamsService_ = nh_.serviceClient<spot_msgs::SetObstacleParams>("/spot/obstacle_params");
  // allowMotionService_ = nh_.serviceClient<std_srvs::SetBool>("/spot/allow_motion");
  // bodyPoseService_ = nh_.serviceClient<spot_msgs::PosedStand>("/spot/posed_stand");
  dockService_ = client_node_->create_client<spot_msgs::srv::Dock>("/spot/dock");
  undockService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/undock");
  selfRightService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/self_right");
  // rollOverRightService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/roll_over_right");
  // rollOverLeftService_ = client_node_->create_client<std_srvs::srv::Trigger>("/spot/roll_over_left");

  // camSetScreenService_ = nh_.serviceClient<spot_cam::SetString>("/spot/cam/set_screen");
  // camSetPTZService_ = nh_.serviceClient<spot_cam::SetPTZState>("/spot/cam/ptz/set_position");
  // camLookAtPointService_ = nh_.serviceClient<spot_cam::LookAtPoint>("/spot/cam/ptz/look_at_point");

  claimLeaseButton = this->findChild<QPushButton*>("claimLeaseButton");
  releaseLeaseButton = this->findChild<QPushButton*>("releaseLeaseButton");
  powerOnButton = this->findChild<QPushButton*>("powerOnButton");
  powerOffButton = this->findChild<QPushButton*>("powerOffButton");
  standButton = this->findChild<QPushButton*>("standButton");
  sitButton = this->findChild<QPushButton*>("sitButton");
  // setBodyPoseButton = this->findChild<QPushButton*>("setBodyPoseButton");
  // setBodyNeutralButton = this->findChild<QPushButton*>("setBodyNeutralButton");
  // setMaxVelButton = this->findChild<QPushButton*>("setMaxVelButton");
  // setGaitButton = this->findChild<QPushButton*>("setGaitButton");
  // setSwingHeightButton = this->findChild<QPushButton*>("setSwingHeightButton");
  // setObstaclePaddingButton = this->findChild<QPushButton*>("setObstaclePaddingButton");
  // setGratedSurfacesButton = this->findChild<QPushButton*>("setGratedSurfacesButton");
  // setFrictionButton = this->findChild<QPushButton*>("setFrictionButton");
  // allowMotionButton = this->findChild<QPushButton*>("allowMotionButton");
  dockButton = this->findChild<QPushButton*>("dockButton");
  undockButton = this->findChild<QPushButton*>("undockButton");
  selfRightButton = this->findChild<QPushButton*>("selfRightButton");
  // rollOverLeftButton = this->findChild<QPushButton*>("rollOverLeftButton");
  // rollOverRightButton = this->findChild<QPushButton*>("rollOverRightButton");

  statusLabel = this->findChild<QLabel*>("statusLabel");
  estimatedRuntimeLabel = this->findChild<QLabel*>("estimatedRuntimeLabel");
  batteryStateLabel = this->findChild<QLabel*>("batteryStateLabel");
  motorStateLabel = this->findChild<QLabel*>("motorStateLabel");
  batteryTempLabel = this->findChild<QLabel*>("batteryTempLabel");
  estopLabel = this->findChild<QLabel*>("estopLabel");

  gaitComboBox = this->findChild<QComboBox*>("gaitComboBox");
  swingHeightComboBox = this->findChild<QComboBox*>("swingHeightComboBox");
  gratedSurfacesComboBox = this->findChild<QComboBox*>("gratedSurfacesComboBox");

  obstaclePaddingSpin = this->findChild<QDoubleSpinBox*>("obstaclePaddingSpin");
  frictionSpin = this->findChild<QDoubleSpinBox*>("frictionSpin");

  dockFiducialSpin = this->findChild<QSpinBox*>("dockFiducialSpin");

  // spot cam
  // setPTZButton = this->findChild<QPushButton*>("setPTZButton");
  // setScreenButton = this->findChild<QPushButton*>("setScreenButton");;
  // chooseScreenComboBox = this->findChild<QComboBox*>("chooseScreenComboBox");
  // choosePTZComboBox = this->findChild<QComboBox*>("choosePTZComboBox");
  // LEDSpinBox = this->findChild<QDoubleSpinBox*>("LEDSpinBox");
  // panSpinBox = this->findChild<QDoubleSpinBox*>("panSpinBox");
  // tiltSpinBox = this->findChild<QDoubleSpinBox*>("tiltSpinBox");
  // zoomSpinBox = this->findChild<QDoubleSpinBox*>("zoomSpinBox");

  // lookAtFrameLineEdit = this->findChild<QLineEdit*>("lookAtFrameLineEdit");
  // lookXSpinBox = this->findChild<QDoubleSpinBox*>("lookXSpinBox");
  // lookYSpinBox = this->findChild<QDoubleSpinBox*>("lookYSpinBox");
  // lookZSpinBox = this->findChild<QDoubleSpinBox*>("lookZSpinBox");
  // imageWidthSpinBox = this->findChild<QDoubleSpinBox*>("imageWidthSpinBox");
  // lookZoomSpinBox = this->findChild<QDoubleSpinBox*>("lookZoomSpinBox");
  // lookAtPointButton = this->findChild<QPushButton*>("lookAtPointButton");
  // trackPointButton = this->findChild<QPushButton*>("trackPointButton");

  setupComboBoxes();
  setupStopButtons();
  setupSpinBoxes();

  // Subscribe to things after everything is set up to avoid crashes when things aren't initialised
  leaseSub_ = client_node_->create_subscription<spot_msgs::msg::LeaseArray>(
      "/spot/status/leases", 1, std::bind(&SpotPanel::leaseCallback, this, std::placeholders::_1));
  estopSub_ = client_node_->create_subscription<spot_msgs::msg::EStopStateArray>(
      "/spot/status/estop", 1, std::bind(&SpotPanel::estopCallback, this, std::placeholders::_1));
  mobilityParamsSub_ = client_node_->create_subscription<spot_msgs::msg::MobilityParams>(
      "/spot/status/mobility_params", 1, std::bind(&SpotPanel::mobilityParamsCallback, this, std::placeholders::_1));
  batterySub_ = client_node_->create_subscription<spot_msgs::msg::BatteryStateArray>(
      "/spot/status/battery_states", 1, std::bind(&SpotPanel::batteryCallback, this, std::placeholders::_1));
  powerSub_ = client_node_->create_subscription<spot_msgs::msg::PowerState>(
      "/spot/status/power_states", 1, std::bind(&SpotPanel::powerCallback, this, std::placeholders::_1));
  // motionAllowedSub_ = nh_.subscribe("/spot/status/motion_allowed", 1, std::bind(&SpotPanel::motionAllowedCallback,
  // this, std::placeholders::_1)); camScreensSub_ = nh_.subscribe("/spot/cam/screens", 1, &SpotPanel::screensCallback,
  // this); camPTZSub_ = nh_.subscribe("/spot/cam/ptz/list", 1, &SpotPanel::ptzCallback, this);

  // camLEDPub_ = nh_.advertise<std_msgs::Float32>("/spot/cam/set_leds", 1);

  connect(claimLeaseButton, SIGNAL(clicked()), this, SLOT(claimLease()));
  connect(releaseLeaseButton, SIGNAL(clicked()), this, SLOT(releaseLease()));
  connect(powerOnButton, SIGNAL(clicked()), this, SLOT(powerOn()));
  connect(powerOffButton, SIGNAL(clicked()), this, SLOT(powerOff()));
  connect(sitButton, SIGNAL(clicked()), this, SLOT(sit()));
  connect(standButton, SIGNAL(clicked()), this, SLOT(stand()));
  // connect(setBodyPoseButton, SIGNAL(clicked()), this, SLOT(sendBodyPose()));
  // connect(setBodyNeutralButton, SIGNAL(clicked()), this, SLOT(sendNeutralBodyPose()));
  // connect(setMaxVelButton, SIGNAL(clicked()), this, SLOT(setMaxVel()));
  connect(releaseStopButton, SIGNAL(clicked()), this, SLOT(releaseStop()));
  connect(hardStopButton, SIGNAL(clicked()), this, SLOT(hardStop()));
  connect(gentleStopButton, SIGNAL(clicked()), this, SLOT(gentleStop()));
  connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
  // connect(setGaitButton, SIGNAL(clicked()), this, SLOT(setGait()));
  // connect(setSwingHeightButton, SIGNAL(clicked()), this, SLOT(setSwingHeight()));
  // connect(setObstaclePaddingButton, SIGNAL(clicked()), this, SLOT(setObstacleParams()));
  // connect(setGratedSurfacesButton, SIGNAL(clicked()), this, SLOT(setTerrainParams()));
  // connect(setFrictionButton, SIGNAL(clicked()), this, SLOT(setTerrainParams()));
  // connect(allowMotionButton, SIGNAL(clicked()), this, SLOT(allowMotion()));
  connect(dockButton, SIGNAL(clicked()), this, SLOT(dock()));
  connect(undockButton, SIGNAL(clicked()), this, SLOT(undock()));
  connect(selfRightButton, SIGNAL(clicked()), this, SLOT(selfRight()));
  // connect(rollOverLeftButton, SIGNAL(clicked()), this, SLOT(rollOverLeft()));
  // connect(rollOverRightButton, SIGNAL(clicked()), this, SLOT(rollOverRight()));
  /*         connect(setPTZButton, SIGNAL(clicked()), this, SLOT(setCamPTZ()));
          connect(setScreenButton, SIGNAL(clicked()), this, SLOT(setCamScreen()));
          connect(LEDSpinBox, SIGNAL(valueChanged(double)), this, SLOT(setCamLED(double)));
          connect(trackPointButton, SIGNAL(clicked()), this, SLOT(camTrackPoint()));
          connect(lookAtPointButton, SIGNAL(clicked()), this, SLOT(camLookAtPoint())); */

  spin_timer.start(200, this);
}

/* This timer event is triggered by the spin_timer and is needed to ensure that the node spins */
void SpotPanel::timerEvent(QTimerEvent* event) {
  if (client_node_ != nullptr) {
    rclcpp::spin_some(client_node_);
  }
}

void SpotPanel::setupStopButtons() {
  stopButton = this->findChild<QPushButton*>("stopButton");
  QPalette pal = stopButton->palette();
  pal.setColor(QPalette::Button, QColor(255, 165, 0));
  stopButton->setAutoFillBackground(true);
  stopButton->setPalette(pal);
  stopButton->update();

  gentleStopButton = this->findChild<QPushButton*>("gentleStopButton");
  pal = gentleStopButton->palette();
  pal.setColor(QPalette::Button, QColor(255, 0, 255));
  gentleStopButton->setAutoFillBackground(true);
  gentleStopButton->setPalette(pal);
  gentleStopButton->update();

  hardStopButton = this->findChild<QPushButton*>("hardStopButton");
  hardStopButton->setText(QString::fromUtf8("\u26A0 Kill Motors"));
  pal = hardStopButton->palette();
  pal.setColor(QPalette::Button, QColor(255, 0, 0));
  hardStopButton->setAutoFillBackground(true);
  hardStopButton->setPalette(pal);
  hardStopButton->update();

  releaseStopButton = this->findChild<QPushButton*>("releaseStopButton");
  pal = releaseStopButton->palette();
  pal.setColor(QPalette::Button, QColor(0, 255, 0));
  releaseStopButton->setAutoFillBackground(true);
  releaseStopButton->setPalette(pal);
  releaseStopButton->update();
}

void SpotPanel::setupComboBoxes() {
  // Iterate over the map for this combobox and add items. By default items in the
  // map are in ascending order by key
  /* for (const auto& item : gaitMap) {
      gaitComboBox->addItem(QString(item.second.c_str()));
  }
  // Disable the unknown entry in the combobox so that it cannot be selected and sent to the service
  QStandardItemModel* model = qobject_cast<QStandardItemModel *>(gaitComboBox->model());
  QStandardItem* item = model->item(0);
  item->setFlags(item->flags() & ~Qt::ItemIsEnabled); */

  /*         for (const auto& item : swingHeightMap) {
              swingHeightComboBox->addItem(QString(item.second.c_str()));
          }
          model = qobject_cast<QStandardItemModel *>(swingHeightComboBox->model());
          item = model->item(0);
          item->setFlags(item->flags() & ~Qt::ItemIsEnabled); */

  /*         for (const auto& item : gratedSurfacesMap) {
              gratedSurfacesComboBox->addItem(QString(item.second.c_str()));
          }
          model = qobject_cast<QStandardItemModel *>(gratedSurfacesComboBox->model());
          item = model->item(0);
          item->setFlags(item->flags() & ~Qt::ItemIsEnabled); */
}

void SpotPanel::setupSpinBoxes() {
  double linearVelocityLimit = 2;
  linearXSpin = this->findChild<QDoubleSpinBox*>("linearXSpin");
  linearXLabel = this->findChild<QLabel*>("linearXLabel");
  updateLabelTextWithLimit(linearXLabel, 0, linearVelocityLimit);
  linearXSpin->setMaximum(linearVelocityLimit);
  linearXSpin->setMinimum(0);

  linearYSpin = this->findChild<QDoubleSpinBox*>("linearYSpin");
  linearYLabel = this->findChild<QLabel*>("linearYLabel");
  updateLabelTextWithLimit(linearYLabel, 0, linearVelocityLimit);
  linearYSpin->setMaximum(linearVelocityLimit);
  linearYSpin->setMinimum(0);

  angularZSpin = this->findChild<QDoubleSpinBox*>("angularZSpin");
  angularZLabel = this->findChild<QLabel*>("angularZLabel");
  updateLabelTextWithLimit(angularZLabel, 0, linearVelocityLimit);
  angularZSpin->setMaximum(linearVelocityLimit);
  angularZSpin->setMinimum(0);

  double bodyHeightLimit = 0.15;
  bodyHeightSpin = this->findChild<QDoubleSpinBox*>("bodyHeightSpin");
  bodyHeightLabel = this->findChild<QLabel*>("bodyHeightLabel");
  updateLabelTextWithLimit(bodyHeightLabel, -bodyHeightLimit, bodyHeightLimit);
  bodyHeightSpin->setMaximum(bodyHeightLimit);
  bodyHeightSpin->setMinimum(-bodyHeightLimit);

  double rollLimit = 20;
  rollSpin = this->findChild<QDoubleSpinBox*>("rollSpin");
  rollLabel = this->findChild<QLabel*>("rollLabel");
  updateLabelTextWithLimit(rollLabel, -rollLimit, rollLimit);
  rollSpin->setMaximum(rollLimit);
  rollSpin->setMinimum(-rollLimit);

  double pitchLimit = 30;
  pitchSpin = this->findChild<QDoubleSpinBox*>("pitchSpin");
  pitchLabel = this->findChild<QLabel*>("pitchLabel");
  updateLabelTextWithLimit(pitchLabel, -pitchLimit, pitchLimit);
  pitchSpin->setMaximum(pitchLimit);
  pitchSpin->setMinimum(-pitchLimit);

  double yawLimit = 30;
  yawSpin = this->findChild<QDoubleSpinBox*>("yawSpin");
  yawLabel = this->findChild<QLabel*>("yawLabel");
  updateLabelTextWithLimit(yawLabel, -yawLimit, yawLimit);
  yawSpin->setMaximum(yawLimit);
  yawSpin->setMinimum(-yawLimit);
}

void SpotPanel::updateLabelTextWithLimit(QLabel* label, double limit_lower, double limit_upper) {
  int precision = 1;
  // Kind of hacky but default to_string returns 6 digit precision which is unnecessary
  std::string limit_lower_value =
      std::to_string(limit_lower).substr(0, std::to_string(limit_lower).find(".") + precision + 1);
  std::string limit_upper_value =
      std::to_string(limit_upper).substr(0, std::to_string(limit_upper).find(".") + precision + 1);
  std::string limit_range = " [" + limit_lower_value + ", " + limit_upper_value + "]";
  std::string current_text = label->text().toStdString();
  label->setText(QString((current_text + limit_range).c_str()));
}

void SpotPanel::setControlButtons() {
  claimLeaseButton->setEnabled(!haveLease);
  releaseLeaseButton->setEnabled(haveLease);
  powerOnButton->setEnabled(haveLease);
  powerOffButton->setEnabled(haveLease);
  sitButton->setEnabled(haveLease);
  standButton->setEnabled(haveLease);
  // setBodyPoseButton->setEnabled(haveLease);
  // setBodyNeutralButton->setEnabled(haveLease);
  // setMaxVelButton->setEnabled(haveLease);
  releaseStopButton->setEnabled(haveLease && isEStopped);
  hardStopButton->setEnabled(haveLease);
  gentleStopButton->setEnabled(haveLease);
  stopButton->setEnabled(haveLease);
  // setGaitButton->setEnabled(haveLease);
  // setSwingHeightButton->setEnabled(haveLease);
  // setObstaclePaddingButton->setEnabled(haveLease);
  // setFrictionButton->setEnabled(haveLease);
  // setGratedSurfacesButton->setEnabled(haveLease);
  // allowMotionButton->setEnabled(haveLease);
  dockButton->setEnabled(haveLease);
  undockButton->setEnabled(haveLease);
  selfRightButton->setEnabled(haveLease);
  // rollOverLeftButton->setEnabled(haveLease);
  // rollOverRightButton->setEnabled(haveLease);
}

/**
 * @brief Call a ros std_msgs/Trigger service
 *
 * Modifies the status label text depending on the result
 *
 * @param service Service to call
 * @param serviceName Name of the service to use in labels
 * @return true if successfully called
 * @return false otherwise
 */
bool SpotPanel::callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service) {
  std_srvs::srv::Trigger::Request req;
  return callCustomTriggerService<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Request>(service, req);
}

template <typename C, typename T>
/**
 * @brief Call an arbitrary service which has a response type of bool, str
 *
 * Modifies the status label text depending on the result
 *
 * @param service Service to call
 * @param serviceName Name of the service to use in labels
 * @param serviceRequest Request to make to the service
 * @return true if successfully called
 * @return false otherwise
 */
bool SpotPanel::callCustomTriggerService(typename rclcpp::Client<C>::SharedPtr service, T serviceRequest) {
  std::string serviceName = std::string(service->get_service_name());
  std::string labelText = "Calling " + serviceName + " service";
  statusLabel->setText(QString(labelText.c_str()));
  auto future = service->async_send_request(std::make_shared<T>(serviceRequest));
  auto future_result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(10));

  if (future_result == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->success) {
      labelText = "Successfully called " + serviceName + " service";
      statusLabel->setText(QString(labelText.c_str()));
      return true;
    } else {
      labelText = serviceName + " service failed";
      statusLabel->setText(QString(labelText.c_str()));
      return false;
    }
  } else if (future_result == rclcpp::FutureReturnCode::TIMEOUT) {
    labelText = "Call to " + serviceName + " timed out";
    statusLabel->setText(QString(labelText.c_str()));
    service->remove_pending_request(future);
    return false;
  } else {
    labelText = "Failed to call " + serviceName + " service";
    statusLabel->setText(QString(labelText.c_str()));
    return false;
  }
}

/**
 * @brief Check held leases and enable or disable buttons accordingly
 *
 * check to see if the body is already owned by the ROS node
 * the resource will be "body" and the lease_owner.client_name will begin with "ros_spot"
 * if the claim exists, treat this as a successful click of the Claim button
 * if the claim does not exist, treat this as a click of the Release button
 *
 * @param leases
 */
void SpotPanel::leaseCallback(const spot_msgs::msg::LeaseArray::ConstSharedPtr& leases) {
  bool msg_has_lease = false;
  for (int i = leases->resources.size() - 1; i >= 0; i--) {
    const spot_msgs::msg::LeaseResource& resource = leases->resources[i];
    bool right_resource = resource.resource.compare("body") == 0;
    bool owned_by_ros = resource.lease_owner.client_name.compare(0, 8, "ros_spot") == 0;

    if (right_resource && owned_by_ros) {
      msg_has_lease = true;
    }
  }

  if (msg_has_lease != haveLease) {
    haveLease = msg_has_lease;
    setControlButtons();
  }
}

/**
 * @brief Check estop state and disable control buttons if estopped
 *
 * @param estops
 */
void SpotPanel::estopCallback(const spot_msgs::msg::EStopStateArray::ConstSharedPtr& estops) {
  bool softwareEstopped = false;
  std::string estopString("E-stops:");
  for (const auto& estop : estops->estop_states) {
    // Can't release hardware estops from the sdk
    bool stopped = false;
    if (estop.state == spot_msgs::msg::EStopState::STATE_ESTOPPED) {
      if (estop.type == spot_msgs::msg::EStopState::TYPE_SOFTWARE) {
        softwareEstopped = true;
      }
      stopped = true;
    }
    std::string stoppedStr(stopped ? "On" : "Off");
    if (estop.name == "hardware_estop") {
      estopString += " hardware: " + stoppedStr;
    } else if (estop.name == "payload_estop") {
      estopString += " payload: " + stoppedStr;
    } else if (estop.name == "software_estop") {
      estopString += " software: " + stoppedStr;
    } else {
      estopString += " " + estop.name + ": " + stoppedStr;
    }
  }

  estopLabel->setText(QString(estopString.c_str()));

  if (softwareEstopped != isEStopped) {
    isEStopped = softwareEstopped;
    setControlButtons();
  }
}

void SpotPanel::mobilityParamsCallback(const spot_msgs::msg::MobilityParams::ConstSharedPtr& params) {
  if (*params == _lastMobilityParams) {
    // If we don't check this, the user will never be able to modify values since they will constantly reset
    return;
  }
  /*         linearXSpin->setValue(params->velocity_limit.linear.x);
          linearYSpin->setValue(params->velocity_limit.linear.y);
          angularZSpin->setValue(params->velocity_limit.angular.z); */

  // Set the combo box values depending on whether there is a nonzero value coming from the params. If there isn't,
  // set the value based on what it is when using the controller
  /*         if (params->locomotion_hint > 0) {
              gaitComboBox->setCurrentIndex(gaitComboBox->findText(gaitMap.at(params->locomotion_hint).c_str()));
          } else {
              gaitComboBox->setCurrentIndex(gaitComboBox->findText(gaitMap.at(spot_msgs::SetLocomotion::Request::HINT_AUTO).c_str()));
          }
          if (params->swing_height > 0) {
              swingHeightComboBox->setCurrentIndex(swingHeightComboBox->findText(swingHeightMap.at(params->swing_height).c_str()));
          } else {
              swingHeightComboBox->setCurrentIndex(swingHeightComboBox->findText(swingHeightMap.at(spot_msgs::SetSwingHeight::Request::SWING_HEIGHT_MEDIUM).c_str()));
          }
          if (params->terrain_params.grated_surfaces_mode > 0) {
              gratedSurfacesComboBox->setCurrentIndex(gratedSurfacesComboBox->findText(gratedSurfacesMap.at(params->terrain_params.grated_surfaces_mode).c_str()));
          } else {
              gratedSurfacesComboBox->setCurrentIndex(gratedSurfacesComboBox->findText(gratedSurfacesMap.at(spot_msgs::TerrainParams::GRATED_SURFACES_MODE_AUTO).c_str()));
          } */

  _lastMobilityParams = *params;
}

void SpotPanel::batteryCallback(const spot_msgs::msg::BatteryStateArray::ConstSharedPtr& battery) {
  spot_msgs::msg::BatteryState battState = battery->battery_states[0];
  std::string estRuntime = "Estimated runtime: " + std::to_string(battState.estimated_runtime.sec / 60) + " min";
  estimatedRuntimeLabel->setText(QString(estRuntime.c_str()));

  auto temps = battState.temperatures;
  if (!temps.empty()) {
    auto minmax = std::minmax_element(temps.begin(), temps.end());
    float total = std::accumulate(temps.begin(), temps.end(), 0);
    // Don't care about float values here
    int tempMin = *minmax.first;
    int tempMax = *minmax.second;
    int tempAvg = total / temps.size();
    std::string battTemp = "Battery temp: min " + std::to_string(tempMin) + ", max " + std::to_string(tempMax) +
                           ", avg " + std::to_string(tempAvg);
    batteryTempLabel->setText(QString(battTemp.c_str()));
  } else {
    batteryTempLabel->setText(QString("Battery temp: No battery"));
  }

  std::string status;
  switch (battState.status) {
    case spot_msgs::msg::BatteryState::STATUS_UNKNOWN:
      status = "Unknown";
      break;
    case spot_msgs::msg::BatteryState::STATUS_MISSING:
      status = "Missing";
      break;
    case spot_msgs::msg::BatteryState::STATUS_CHARGING:
      status = "Charging";
      break;
    case spot_msgs::msg::BatteryState::STATUS_DISCHARGING:
      status = "Discharging";
      break;
    case spot_msgs::msg::BatteryState::STATUS_BOOTING:
      status = "Booting";
      break;
    default:
      status = "Invalid";
      break;
  }

  std::string battStatusStr;
  if (battState.status == spot_msgs::msg::BatteryState::STATUS_CHARGING ||
      battState.status == spot_msgs::msg::BatteryState::STATUS_DISCHARGING) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(0) << battState.charge_percentage;
    std::string pct = stream.str() + "%";
    stream.str("");
    stream.clear();
    stream << std::fixed << std::setprecision(1) << battState.voltage;
    std::string volt = stream.str() + "V";
    stream.str("");
    stream.clear();
    stream << battState.current;
    std::string amp = stream.str() + "A";
    battStatusStr = "Battery state: " + status + ", " + pct + ", " + volt + ", " + amp;
  } else {
    battStatusStr = "Battery state: " + status;
  }
  batteryStateLabel->setText(QString(battStatusStr.c_str()));
}

void SpotPanel::powerCallback(const spot_msgs::msg::PowerState::ConstSharedPtr& power) {
  std::string state;
  switch (power->motor_power_state) {
    case spot_msgs::msg::PowerState::STATE_POWERING_ON:
      state = "Powering on";
      powerOnButton->setEnabled(false);
      break;
    case spot_msgs::msg::PowerState::STATE_POWERING_OFF:
      state = "Powering off";
      powerOffButton->setEnabled(false);
      sitButton->setEnabled(false);
      standButton->setEnabled(false);
      break;
    case spot_msgs::msg::PowerState::STATE_ON:
      state = "On";
      powerOnButton->setEnabled(false);
      powerOffButton->setEnabled(true);
      sitButton->setEnabled(true);
      standButton->setEnabled(true);
      break;
    case spot_msgs::msg::PowerState::STATE_OFF:
      state = "Off";
      powerOnButton->setEnabled(true && haveLease);
      powerOffButton->setEnabled(false);
      sitButton->setEnabled(false);
      standButton->setEnabled(false);
      break;
    case spot_msgs::msg::PowerState::STATE_ERROR:
      state = "Error";
      break;
    case spot_msgs::msg::PowerState::STATE_UNKNOWN:
      state = "Unknown";
      break;
    default:
      state = "Invalid";
  }
  std::string motorState = "Motor state: " + state;
  motorStateLabel->setText(QString(motorState.c_str()));
}

// void SpotPanel::motionAllowedCallback(const std_msgs::msg::Bool& motion_allowed) {
//   motionAllowed = motion_allowed.data;
//   if (!motion_allowed.data) {
//     stopButton->setText("Motion is disallowed");
//     stopButton->setEnabled(false);
//     allowMotionButton->setText("Allow motion");
//   } else {
//     stopButton->setText("Stop");
//     if (haveLease) {
//       stopButton->setEnabled(true);
//     }
//     allowMotionButton->setText("Disallow motion");
//   }
// }

void SpotPanel::sit() {
  callTriggerService(sitService_);
}

void SpotPanel::stand() {
  callTriggerService(standService_);
}

void SpotPanel::powerOn() {
  callTriggerService(powerOnService_);
}

void SpotPanel::powerOff() {
  callTriggerService(powerOffService_);
}

void SpotPanel::claimLease() {
  if (callTriggerService(claimLeaseService_)) claimLeaseButton->setEnabled(false);
}

void SpotPanel::releaseLease() {
  if (callTriggerService(releaseLeaseService_)) releaseLeaseButton->setEnabled(false);
}

void SpotPanel::stop() {
  callTriggerService(stopService_);
}

/*     void SpotPanel::allowMotion() {
        std_srvs::srv::SetBool::Request req;
        req.data = !motionAllowed;
        callCustomTriggerService(allowMotionService_, req);
    } */

void SpotPanel::hardStop() {
  callTriggerService(hardStopService_);
}

void SpotPanel::gentleStop() {
  callTriggerService(gentleStopService_);
}

void SpotPanel::releaseStop() {
  callTriggerService(releaseStopService_);
}

/*     void SpotPanel::setMaxVel() {
        spot_msgs::msg::SetVelocity::Request req;
        req.velocity_limit.angular.z = angularZSpin->value();
        req.velocity_limit.linear.x = linearXSpin->value();
        req.velocity_limit.linear.y = linearYSpin->value();
        callCustomTriggerService(maxVelocityService_, req);
    } */
/*
    void SpotPanel::sendBodyPose() {
        spot_msgs::srv::PosedStand::Request req;
        req.body_height = bodyHeightSpin->value();
        req.body_yaw = yawSpin->value();
        req.body_pitch = pitchSpin->value();
        req.body_roll = rollSpin->value();

        callCustomTriggerService(bodyPoseService_, req);
    } */

/*     void SpotPanel::sendNeutralBodyPose() {
        ROS_INFO("Sending body neutral pose");
        spot_msgs::srv::PosedStand::Request req;
        callCustomTriggerService(bodyPoseService_, req);
    } */

/**
 * @brief Get the message constant integer that corresponds to the currently selected combobox item
 *
 * @param comboBox Combobox whose selection should be checked
 * @param comboBoxMap Mapping from message constants to text in the combobox
 * @return int > 0 indicating the map constant, or -1 if it couldn't be found
 */
int comboBoxSelectionToMessageConstantInt(QComboBox* comboBox, const std::map<uint, std::string>& comboBoxMap) {
  std::string selectionText = comboBox->currentText().toStdString();
  for (const auto& item : comboBoxMap) {
    if (selectionText == item.second) {
      return item.first;
    }
  }
  return -1;
}

/*     void SpotPanel::setGait() {
        spot_msgs::SetLocomotion req;
        req.request.locomotion_mode = comboBoxSelectionToMessageConstantInt(gaitComboBox, gaitMap);
        callCustomTriggerService(gaitService_, "set gait", req);
    }

    void SpotPanel::setSwingHeight() {
        spot_msgs::SetSwingHeight req;
        req.request.swing_height = comboBoxSelectionToMessageConstantInt(swingHeightComboBox, swingHeightMap);
        callCustomTriggerService(swingHeightService_, "set swing height", req);
    }

    void SpotPanel::setTerrainParams() {
        spot_msgs::SetTerrainParams req;
        req.request.terrain_params.grated_surfaces_mode = comboBoxSelectionToMessageConstantInt(gratedSurfacesComboBox,
   gratedSurfacesMap); req.request.terrain_params.ground_mu_hint = frictionSpin->value();
        callCustomTriggerService(terrainParamsService_, "set terrain params", req);
    }

    void SpotPanel::setObstacleParams() {
        spot_msgs::SetObstacleParams req;
        req.request.obstacle_params.obstacle_avoidance_padding = obstaclePaddingSpin->value();
        callCustomTriggerService(obstacleParamsService_, "set obstacle params", req);
    } */

void SpotPanel::undock() {
  callTriggerService(undockService_);
}

void SpotPanel::dock() {
  spot_msgs::srv::Dock::Request req;
  req.dock_id = dockFiducialSpin->value();
  callCustomTriggerService<spot_msgs::srv::Dock, spot_msgs::srv::Dock::Request>(dockService_, req);
}

void SpotPanel::selfRight() {
  callTriggerService(selfRightService_);
}

/*
void SpotPanel::rollOverLeft() {
    callTriggerService(rollOverLeftService_);
}

void SpotPanel::rollOverRight() {
    callTriggerService(rollOverRightService_);
} */

/*     void SpotPanel::ptzCallback(const spot_cam::PTZDescriptionArray &ptz_descriptions) {
        // Combobox items should only be populated a single time. The ptzs do not change unless the camera changes
        if (choosePTZComboBox->count() != 0) {
            return;
        }
        for (auto p: ptz_descriptions.ptzs) {
            choosePTZComboBox->addItem(p.name.c_str());
        }
    }

    void SpotPanel::screensCallback(const spot_cam::StringMultiArray& screens) {
        // Combobox items should only be populated a single time. The screens do not change unless the camera changes
        if (chooseScreenComboBox->count() != 0) {
            return;
        }
        for (auto s: screens.data) {
            chooseScreenComboBox->addItem(s.c_str());
        }
    }

    void SpotPanel::setCamPTZ() {
        spot_cam::PTZDescription ptz_desc;
        ptz_desc.name = choosePTZComboBox->currentText().toStdString();
        spot_cam::SetPTZState req;
        req.request.command.ptz = ptz_desc;
        req.request.command.pan = panSpinBox->value();
        req.request.command.tilt = tiltSpinBox->value();
        req.request.command.zoom = zoomSpinBox->value();
        callCustomTriggerService(camSetPTZService_, "set cam PTZ", req);
    }

    void SpotPanel::setCamScreen() {
        spot_cam::SetString req;
        req.request.value = chooseScreenComboBox->currentText().toStdString();
        callCustomTriggerService(camSetScreenService_, "set cam screen", req);
    }

    void SpotPanel::setCamLED(double value) {
        std_msgs::Float32 value_ros;
        value_ros.data = value;
        camLEDPub_.publish(value_ros);
    }

    void SpotPanel::camTrackPoint() {
        lookAtPoint(true);
    }

    void SpotPanel::camLookAtPoint() {
        lookAtPoint(false);
    }

    void SpotPanel::lookAtPoint(const bool track) {
        spot_cam::LookAtPoint req;
        req.request.track = track;
        req.request.target.header.frame_id = lookAtFrameLineEdit->text().toStdString();
        req.request.target.point.x = lookXSpinBox->value();
        req.request.target.point.y = lookYSpinBox->value();
        req.request.target.point.z = lookZSpinBox->value();
        req.request.image_width = imageWidthSpinBox->value();
        req.request.zoom_level = zoomSpinBox->value();

        callCustomTriggerService(camLookAtPointService_, "cam look at point", req);
    } */

void SpotPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void SpotPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
}
}  // end namespace spot_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(spot_rviz_plugins::SpotPanel, rviz_common::Panel)
