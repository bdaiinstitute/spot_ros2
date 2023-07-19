#!/bin/bash -e

SCRIPT=${BASH_SOURCE[0]}
SCRIPT_PATH="$(dirname "$SCRIPT")"
cd $SCRIPT_PATH

# Builds the image
docker build -t openvpn_client -f Dockerfile .

# Exports the image, uses pigz
docker save openvpn_client | pigz > openvpn_client.tar.gz

# tar -cvzf openvpn_client.spx \
#     openvpn_client.tar.gz \
#     manifest.json \
#     docker-compose.yml \
#     icon.png

# Cleanup intermediate image
# rm openvpn_client.tar.gz