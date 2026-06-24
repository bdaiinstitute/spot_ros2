# Copyright (c) 2023-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.
# Result codes for the grasping pipeline.

# The entire grasp pipeline succeeded and the final hand pose is "close" to the
# requested retract pose.
int32 SUCCESS                  = 0

# Result if the move to the pregrasp pose fails.
int32 PREPARE_TO_GRASP_FAILED  = 1

# Result if the move to the retract pose fails.
int32 RETRACT_FAILED           = 2

# Result if the move to the pregrasp pose fails.
int32 OPEN_GRIPPER_FAILED      = 3

# Result if the move to the pregrasp pose fails.
int32 CLOSE_GRIPPER_FAILED     = 4

# Failure if a transform could not be looked up.
int32 TRANSFORM_LOOKUP_FAILURE = 5

# Failures not to do with grasping; should be accounted for in separate result
# codes that account for middleware connections, transforms, hardware (whether
# or not Spot has an arm), etc.
int32 OTHER_FAILURE            = 6

# Result code to be set.
int32 result_code
