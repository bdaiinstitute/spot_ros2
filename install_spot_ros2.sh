ARM=false
ROS_DISTRO=humble
HELP=$'--arm64: Installs ARM64 version'
REQUIREMENTS_FILE=spot_wrapper/requirements.txt

while true; do
  case "$1" in
    --arm64 ) ARM=true; shift ;;
    -h | --help ) echo "$HELP"; exit 0;;
    -- ) shift; break ;;
    * ) break ;;
  esac
done

if test -f "$REQUIREMENTS_FILE"; then
    sudo pip3 install -r $REQUIREMENTS_FILE
else
    echo "ERROR: $REQUIREMENTS_FILE not found. Please initialize spot_wrapper with: git submodule init --update"  
    exit 1
fi

# Install ROS dependencies
# TODO(jschornak-bdai): use rosdep to install these packages by parsing dependencies listed in package.xml
sudo apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-depth-image-proc ros-$ROS_DISTRO-tl-expected
# Install the dist-utils
sudo apt-get install python3-distutils
sudo apt-get install python3-apt
sudo pip3 install --force-reinstall -v "setuptools==59.6.0"


if $ARM; then
    # Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
    wget -q -O /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/bosdyn_msgs-v.3.2.0-humble-arm64/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb
    sudo dpkg -i /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb
    rm /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb

    # Install spot-cpp-sdk
    wget -q -O /tmp/spot-cpp-sdk_3.3.0_arm64.deb https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/v3.3.0-cmake-infra/spot-cpp-sdk_3.3.0_arm64.deb
    sudo dpkg -i /tmp/spot-cpp-sdk_3.3.0_arm64.deb
    rm /tmp/spot-cpp-sdk_3.3.0_arm64.deb
else
    # Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
    wget -q -O /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v3.2.0-frametreesnapshot/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb
    sudo dpkg -i /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb
    rm /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb
    
    # Install spot-cpp-sdk
    wget -q -O /tmp/spot-cpp-sdk_3.3.0_amd64.deb https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/v3.3.0-cmake-infra/spot-cpp-sdk_3.3.0_amd64.deb
    sudo dpkg -i /tmp/spot-cpp-sdk_3.3.0_amd64.deb
    rm /tmp/spot-cpp-sdk_3.3.0_amd64.deb
fi
