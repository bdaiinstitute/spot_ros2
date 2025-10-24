#include <spot_rviz_plugins/spot_panel.hpp>

#include <string.h>
#include <QDoubleValidator>
#include <QFile>
#include <QStandardItemModel>
#include <QUiLoader>
#include <QVBoxLayout>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <spot_msgs/msg/e_stop_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

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

  claimLeaseButton = this->findChild<QPushButton*>("claimLeaseButton");
  releaseLeaseButton = this->findChild<QPushButton*>("releaseLeaseButton");
  powerOnButton = this->findChild<QPushButton*>("powerOnButton");
  powerOffButton = this->findChild<QPushButton*>("powerOffButton");
  standButton = this->findChild<QPushButton*>("standButton");
  sitButton = this->findChild<QPushButton*>("sitButton");
  dockButton = this->findChild<QPushButton*>("dockButton");
  undockButton = this->findChild<QPushButton*>("undockButton");
  selfRightButton = this->findChild<QPushButton*>("selfRightButton");
  setRobotNameButton = this->findChild<QPushButton*>("setRobotNameButton");

  statusLabel = this->findChild<QLabel*>("statusLabel");
  estimatedRuntimeLabel = this->findChild<QLabel*>("estimatedRuntimeLabel");
  batteryStateLabel = this->findChild<QLabel*>("batteryStateLabel");
  motorStateLabel = this->findChild<QLabel*>("motorStateLabel");
  batteryTempLabel = this->findChild<QLabel*>("batteryTempLabel");
  estopLabel = this->findChild<QLabel*>("estopLabel");
  robotNameLineEdit = this->findChild<QLineEdit*>("robotNameLineEdit");

  dockFiducialSpin = this->findChild<QSpinBox*>("dockFiducialSpin");

  setupStopButtons();
  setupSpinBoxes();
  initialiseRosComponents();

  connect(claimLeaseButton, SIGNAL(clicked()), this, SLOT(claimLease()));
  connect(releaseLeaseButton, SIGNAL(clicked()), this, SLOT(releaseLease()));
  connect(powerOnButton, SIGNAL(clicked()), this, SLOT(powerOn()));
  connect(powerOffButton, SIGNAL(clicked()), this, SLOT(powerOff()));
  connect(sitButton, SIGNAL(clicked()), this, SLOT(sit()));
  connect(standButton, SIGNAL(clicked()), this, SLOT(stand()));
  connect(releaseStopButton, SIGNAL(clicked()), this, SLOT(releaseStop()));
  connect(hardStopButton, SIGNAL(clicked()), this, SLOT(hardStop()));
  connect(gentleStopButton, SIGNAL(clicked()), this, SLOT(gentleStop()));
  connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
  connect(dockButton, SIGNAL(clicked()), this, SLOT(dock()));
  connect(undockButton, SIGNAL(clicked()), this, SLOT(undock()));
  connect(selfRightButton, SIGNAL(clicked()), this, SLOT(selfRight()));
  connect(setRobotNameButton, SIGNAL(clicked()), this, SLOT(setRobotName()));

  spin_timer.start(200, this);
}

void SpotPanel::initialiseRosComponents() {
  std::string robot_name = robotNameLineEdit->text().toStdString();

  // If there's no robot name, then everything is in the root namespace
  std::string prefix = "";
  if (robot_name != "") {
    // Otherwise, the services and topics will be in the robot name namespace
    prefix = "/" + robot_name;
  }

  sitService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/sit");
  standService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/stand");
  claimLeaseService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/claim");
  releaseLeaseService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/release");
  powerOnService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/power_on");
  powerOffService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/power_off");
  maxVelocityService_ = client_node_->create_client<spot_msgs::srv::SetVelocity>(prefix + "/velocity_limit");
  hardStopService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/estop/hard");
  gentleStopService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/estop/gentle");
  releaseStopService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/estop/release");
  stopService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/stop");
  dockService_ = client_node_->create_client<spot_msgs::srv::Dock>(prefix + "/dock");
  undockService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/undock");
  selfRightService_ = client_node_->create_client<std_srvs::srv::Trigger>(prefix + "/self_right");
  leaseSub_ = client_node_->create_subscription<spot_msgs::msg::LeaseArray>(
      prefix + "/status/leases", 1, std::bind(&SpotPanel::leaseCallback, this, std::placeholders::_1));
  estopSub_ = client_node_->create_subscription<spot_msgs::msg::EStopStateArray>(
      prefix + "/status/estop", 1, std::bind(&SpotPanel::estopCallback, this, std::placeholders::_1));
  mobilityParamsSub_ = client_node_->create_subscription<spot_msgs::msg::MobilityParams>(
      prefix + "/status/mobility_params", 1,
      std::bind(&SpotPanel::mobilityParamsCallback, this, std::placeholders::_1));
  batterySub_ = client_node_->create_subscription<spot_msgs::msg::BatteryStateArray>(
      prefix + "/status/battery_states", rclcpp::SensorDataQoS(),
      std::bind(&SpotPanel::batteryCallback, this, std::placeholders::_1));
  powerSub_ = client_node_->create_subscription<spot_msgs::msg::PowerState>(
      prefix + "/status/power_states", 1, std::bind(&SpotPanel::powerCallback, this, std::placeholders::_1));
}

void SpotPanel::destroyRosComponents() {
  // Resetting the shared pointer should destroy all the services and subscribers since we only hold one reference.
  sitService_.reset();
  standService_.reset();
  claimLeaseService_.reset();
  releaseLeaseService_.reset();
  powerOnService_.reset();
  powerOffService_.reset();
  maxVelocityService_.reset();
  hardStopService_.reset();
  gentleStopService_.reset();
  releaseStopService_.reset();
  stopService_.reset();
  dockService_.reset();
  undockService_.reset();
  selfRightService_.reset();
  leaseSub_.reset();
  estopSub_.reset();
  mobilityParamsSub_.reset();
  batterySub_.reset();
  powerSub_.reset();
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
  releaseStopButton->setEnabled(haveLease && isEStopped);
  hardStopButton->setEnabled(haveLease);
  gentleStopButton->setEnabled(haveLease);
  stopButton->setEnabled(haveLease);
  dockButton->setEnabled(haveLease);
  undockButton->setEnabled(haveLease);
  selfRightButton->setEnabled(haveLease);
}

/**
 * @brief Call a ros std_msgs/Trigger service
 *
 * Modifies the status label text depending on the result
 *
 * @param service Service to call
 * @param serviceName Name of the service to use in labels
 */
void SpotPanel::callTriggerService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service) {
  std_srvs::srv::Trigger::Request req;
  return callCustomTriggerService<std_srvs::srv::Trigger>(service, req);
}

template <typename C>
void SpotPanel::serviceResponseCallback(std::string serviceName, typename rclcpp::Client<C>::SharedFuture future) {
  std::string labelText;
  auto result = future.get();
  if (result->success) {
    labelText = serviceName + " service call was successful";
  } else {
    labelText = serviceName + " call failed";
  }

  statusLabel->setText(QString(labelText.c_str()));
}

template <typename C>
/**
 * @brief Call an arbitrary service which has a response type of bool, str
 *
 * Modifies the status label text depending on the result
 *
 * @param service Service to call
 * @param serviceName Name of the service to use in labels
 * @param serviceRequest Request to make to the service
 */
void SpotPanel::callCustomTriggerService(typename rclcpp::Client<C>::SharedPtr service,
                                         const typename C::Request& serviceRequest) {
  std::string serviceName = service->get_service_name();
  std::string labelText = "Calling " + serviceName + " service";
  if (!service->wait_for_service(std::chrono::seconds(1))) {
    statusLabel->setText((serviceName + " service didn't exist.").c_str());
    return;
  }

  statusLabel->setText(QString(labelText.c_str()));

  // Have to make this a shared pointer in order to correctly cancel the pending request in the timer callback
  auto future_and_id =
      service->async_send_request(std::make_shared<typename C::Request>(serviceRequest),
                                  [this, serviceName](typename rclcpp::Client<C>::SharedFuture future) {
                                    this->serviceResponseCallback<C>(serviceName, future);
                                  });
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
    bool owned_by_ros = false;
    auto name_pos = resource.lease_owner.client_name.find("ros_spot");
    if (name_pos != std::string::npos) {
      owned_by_ros = true;
    }

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
  callTriggerService(claimLeaseService_);
}

void SpotPanel::releaseLease() {
  callTriggerService(releaseLeaseService_);
}

void SpotPanel::stop() {
  callTriggerService(stopService_);
}

void SpotPanel::hardStop() {
  callTriggerService(hardStopService_);
}

void SpotPanel::gentleStop() {
  callTriggerService(gentleStopService_);
}

void SpotPanel::releaseStop() {
  callTriggerService(releaseStopService_);
}

void SpotPanel::setMaxVel() {
  spot_msgs::srv::SetVelocity::Request req;
  req.velocity_limit.angular.z = angularZSpin->value();
  req.velocity_limit.linear.x = linearXSpin->value();
  req.velocity_limit.linear.y = linearYSpin->value();
  callCustomTriggerService<spot_msgs::srv::SetVelocity>(maxVelocityService_, req);
}

void SpotPanel::undock() {
  callTriggerService(undockService_);
}

void SpotPanel::dock() {
  spot_msgs::srv::Dock::Request req;
  req.dock_id = dockFiducialSpin->value();
  callCustomTriggerService<spot_msgs::srv::Dock>(dockService_, req);
}

void SpotPanel::selfRight() {
  callTriggerService(selfRightService_);
}

void SpotPanel::setRobotName() {
  // When setting the robot name, destroy the ros components properly then reinitialise them.
  destroyRosComponents();
  initialiseRosComponents();
}

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
