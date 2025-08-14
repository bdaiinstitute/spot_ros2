#!/bin/bash

# Source this to set up the Spot Ros 2 environment
# usage: souce setup.bash [robot_name]
#
# Example: source setup.bash tusker

# --- Robot Config ---
#
# Add your Spot robots environment
# can add more robots by following the same pattern

if [ -z "$1" ]; then
  echo "Usage: source setup.bash [robot_name]"
  echo "Available robots: gouger, tusker, rooter, snouter"
  return 1
fi

ROBOT_NAME=$1
echo "Setting up environment for: $(ROBOT_NAME)"
if [ "${ROBOT_NAME}" = "gouger" ]; then
  export SPOT_IP="128.148.140.21"
elif [ "${ROBOT_NAME}" = "tusker" ]; then
  export SPOT_IP="128.148.140.22"
elif [ "${ROBOT_NAME}" = "rooter" ]; then
  export SPOT_IP="128.148.140.23"
elif [ "${ROBOT_NAME}" = "snouter" ]; then
  export SPOT_IP="128.148.140.20"
else
  echo "Unknown robot: ${ROBOT_NAME}"
  return 1
fi

echo "SPOT_IP set to: ${SPOT_IP}"

# --- Workspace source ---

# Check if the install directory exists
if [ -f "install/setup.bash" ]; then
  echo "Sourcing ROS2 workspace"
  source "install/setup.bash"
else
  echo "Workspace not built yet. Run 'colcon build' first."
  return 1
fi

# --- environment varialbe authenticatoin ---

if [ -z "$BOSDYN_CLIENT_USERNAME" ]; then
  echo "Reminder: Set the BOSDYN_CLIENT_USERNAME environment variable."
fi

if [ -z "$BOSDYN_CLIENT_PASSWORD" ]; then
  echo "Reminder: Set the BOSDYN_CLIENT_PASSWORD environment variable"
fi

echo "Setup complete"
