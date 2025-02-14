# This file interfaces with spot to send action. Helper files are robot_commander and simple_spot_commander
import os
from typing import List
from contextlib import closing
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray

from synchros2.subscription import Subscription
import synchros2.process as ros_process
from .robot_commander import RobotCommander
from ament_index_python.packages import get_package_share_directory


@ros_process.main()
def main() -> None:
    print("here")
    robot_name= os.getenv("ROBOT_NAME")  # Get robot name from environment variable
    robotcommander = RobotCommander(robot_name=robot_name)
    
    robotcommander._logger.info(f"name:{robotcommander._robot_name}")
    result = robotcommander.initialize_robot()

    if result is False:
        robotcommander._logger.info("Failed to initialize robot")
        return 
    robotcommander._logger.info("Initialized robot")


    topic_data = Subscription(Pose, "/pose_commands")

    latest_message = None  # Buffer to store the latest message
    is_busy = False  # Flag to indicate if the robot is busy executing an action

    with closing(topic_data.stream()) as stream:
        for message in stream:
            if is_busy:
                # If the robot is busy, buffer the latest message
                latest_message = message
                robotcommander._logger.info("Robot is busy, buffering the latest message")
            else:
                is_busy = True
                result = robotcommander.walk_forward_with_world_frame_goal(message)
                # robotcommander._logger(result.success)
                is_busy = False  # Reset the busy flag after the action is completed

                # Check if there is a buffered message and process it
                if latest_message is not None:
                    is_busy = True
                    result = robotcommander.walk_forward_with_world_frame_goal(latest_message)
                    # robotcommander._logger(result.success)
                    is_busy = False
                    latest_message = None  # Clear the buffer after processing

if __name__ == "__main__":
    main()