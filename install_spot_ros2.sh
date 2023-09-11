ARM=false
ROS_DISTRO=humble
HELP=$'--arm64: Installs ARM64 version'
REQUIREMENTS_FILE=spot_wrapper/requirements.txt
BD_MSGS=ros-humble-bosdyn-msgs

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
    echo "ERROR: $REQUIREMENTS_FILE not found. Please initialize spot_wrapper with:"
    echo "git submodule init"
    echo "git submodule update"  
    exit 1
fi

# Install ROS dependencies
sudo apt install -y ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-depth-image-proc
# Install the dist-utils
sudo apt-get install python3-distutils
sudo apt-get install python3-apt
sudo pip3 install --force-reinstall -v "setuptools==59.6.0"

# Check if bosdyn msgs is installed; if so, then tells you to consider updating
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $BD_MSGS|grep "install ok installed")
# echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "$BD_MSGS not found; going to install"
  if $ARM; then
    # Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
    wget -q -O /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/bosdyn_msgs-v.3.2.0-humble-arm64/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb
  else
      # Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
      wget -q -O /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v3.2.0-frametreesnapshot/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb
  fi
  sudo apt-get update -y
  sudo apt --fix-broken install /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb -y
  rm /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb
  sudo apt-get -f install
  # sudo apt-get --yes install $REQUIRED_PKG
else
  echo "Package $BD_MSGS already installed; for the newest release, consider install using:"
  echo "(AMD64 platform) wget -q -O /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/bosdyn_msgs-v.3.2.0-humble-arm64/ros-humble-bosdyn-msgs_3.2.0-0jammy_arm64.deb"
  echo "(ARM platform) wget -q -O /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/v3.2.0-frametreesnapshot/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb"
  echo "sudo apt-get update -y"
  echo "sudo apt --fix-broken install --reinstall /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb -y"
  echo "rm /tmp/ros-humble-bosdyn-msgs_3.2.0-0jammy_amd64.deb"
  echo "sudo apt-get -f install"
fi

