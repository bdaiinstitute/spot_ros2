# Testing ROS2 images for pub/sub

## Pubbing
Run `python multiple_node_publisher.py --cam <cam_type>`. Cam_types are camera, depth, and depth_registered
## Subbing
Run `python multiple_node_subscriber.py --pub_node <Name of the node publishing, whihc follows the format of (camera_type)_image_publisher> --window_size 100` to see the frequencies printed to the terminal 