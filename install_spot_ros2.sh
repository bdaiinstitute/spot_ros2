ARCH="amd64"
SDK_VERSION="5.0.0"
MSG_VERSION="${SDK_VERSION}"
HELP=$'--arm64: Installs ARM64 version'
REQUIREMENTS_FILE=spot_wrapper/requirements.txt

while true; do
  case "$1" in
    --arm64 | --aarch64 ) ARCH="arm64"; shift ;;
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

sudo apt-get update

# Install ROS dependencies
sudo apt-get install -y python3-rosdep
#NOTE: Initialize only if a sources list definition doesn't exist yet - avoids the rosdep error message
if ! [[ $(ls /etc/ros/rosdep/sources.list.d/*default.list 2> /dev/null) ]]; then
  sudo rosdep init
fi
source /opt/ros/humble/setup.bash && rosdep update && rosdep install --from-paths ./ --ignore-src -y -r --rosdistro=humble

# Install the dist-utils
sudo apt-get install -y python3-distutils
sudo apt-get install -y python3-apt
sudo pip3 install --force-reinstall -v "setuptools==59.6.0"

# Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
wget -q -O /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/${MSG_VERSION}/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run
chmod +x /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run
yes | sudo /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run  --nox11
rm /tmp/ros-humble-bosdyn_msgs_${MSG_VERSION}-jammy_${ARCH}.run

# Install spot-cpp-sdk
wget -q -O /tmp/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/v${SDK_VERSION}/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb
sudo dpkg -i /tmp/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb
rm /tmp/spot-cpp-sdk_${SDK_VERSION}_${ARCH}.deb
