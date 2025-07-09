#!/usr/bin/env bash

# Usage: source set_spot_env.sh config.yaml

if [[ $# -ne 1 ]]; then
  echo "Usage: source $0 <config.yaml>"
  return 1 2>/dev/null || exit 1
fi

YAML_FILE="$1"

# Get the first uncommented hostname line
HOSTNAME_LINE=$(grep -E '^[[:space:]]*hostname:[[:space:]]*".*"' "$YAML_FILE" | head -n 1)

if [[ -z "$HOSTNAME_LINE" ]]; then
  echo "Error: No uncommented hostname entry found in $YAML_FILE"
  return 1 2>/dev/null || exit 1
fi

# Extract IP from the line
SPOT_IP=$(echo "$HOSTNAME_LINE" | sed -E 's/.*hostname:[[:space:]]*"([^"]*)".*/\1/')

# Extract label if present in comment
LABEL=$(echo "$HOSTNAME_LINE" | grep -oE '#[[:space:]]*[^#]*$' | sed 's/#\s*//')

# Extract other credentials
USERNAME=$(grep -E '^[[:space:]]*username:[[:space:]]*".*"' "$YAML_FILE" | sed -E 's/.*username:[[:space:]]*"([^"]*)".*/\1/')
PASSWORD=$(grep -E '^[[:space:]]*password:[[:space:]]*".*"' "$YAML_FILE" | sed -E 's/.*password:[[:space:]]*"([^"]*)".*/\1/')

# Set environment variables
export BOSDYN_CLIENT_USERNAME="$USERNAME"
export BOSDYN_CLIENT_PASSWORD="$PASSWORD"
export SPOT_IP="$SPOT_IP"

# Output
if [[ -n "$LABEL" ]]; then
  echo "Set SPOT_IP=$SPOT_IP  # $LABEL"
else
  echo "Set SPOT_IP=$SPOT_IP"
fi
