#!/bin/bash
set -e

# Source the ros2 spot environment,
# Note: The SPOT_WS environment variable was set in the dockerfile
source "$SPOT_WS/install/setup.bash"

exec "$@"
