ROS_DISTRO=humble  # only humble is currently supported by install script
# Install BD API
pip3 install bosdyn-client bosdyn-mission bosdyn-api bosdyn-core
# Install ROS dependencies
sudo apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-depth-image-proc
# Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
wget -q -O /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v0.0.0-humble/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
sudo dpkg -i /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
rm /tmp/ros-humble-bosdyn-msgs_0.0.0-0jammy_amd64.deb
