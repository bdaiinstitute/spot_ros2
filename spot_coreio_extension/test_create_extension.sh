#!/bin/bash

# Build the image
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
echo "-> Building arm64 docker image"
docker build -t spot_ros2:arm64 --platform linux/arm64/v8 -f Dockerfile.arm64 .

# Save the image, uses pigz
echo "-> Saving the docker image to a tgz file"
docker save spot_ros2:arm64 | pigz > spot_ros2_arm64.tgz

# # Package into extension
# echo "-> Creating Spot Extension"
# tar -cvzf test_spot_ros2_driver.spx \
#     manifest_copy.json \
#     docker-compose.yml \
#     icon.png

# # Cleanup intermediate iamge
# echo "-> Cleaning up"
# # rm spot_ros2_arm64.tgz
echo "-> DONE"
