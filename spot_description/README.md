# spot_description

This ROS 2 package contains the URDF files for Spot. There are two Spot models, referred to as `spot` and `spot_simple`, which primarily differ in the number of links in the base frame. There is also a `standalone_arm` model to view the Spot Arm independently. 

To get the plain URDF files, run the following commands in the `spot_description` directory:

```
# for spot
ros2 run xacro xacro -o ./urdf/out/spot.urdf ./urdf/spot.urdf.xacro

# for simple spot
ros2 run xacro xacro -o ./urdf/out/spot_simple.urdf ./urdf/spot_simple.urdf.xacro

# for standalone arm
ros2 run xacro xacro -o ./urdf/out/standalone_arm.urdf ./urdf/standalone_arm.urdf.xacro
```

The following launchfiles can be used to visualize the Spot models and inspect the joint range.

```
# for Spot without arm
ros2 launch spot_description description.launch.py

# for Spot with arm
ros2 launch spot_description description.launch.py arm:=True

# for only the arm
ros2 launch spot_description standalone_arm.launch.py
```

## Model Properties

The inertial properties of Spot and its arm are extracted from `Isaac Sim`, which based on the collision geometry and uniform mass distribution, defines the mass, CoM, and inertia tensor around the CoM frame. The inertia tensors are around their principal axes. Therefore, the CoM frames are rotated. 

For the arm, we use the following mass values:
+ `body` = 32.86
+ `hip` = 1.68
+ `uleg` = 2.34
+ `lleg` = 0.35
+ `arm_link_sh0` + `arm_link_sh1` = 2.596 Kg (sh0 : 90%, sh1 : 10%)
+ `arm_link_hr0` = 0.0
+ `arm_link_el0` + `arm_link_el1` = 1.450 Kg (el0 : 50%, el1 : 50%)
+ `arm_link_wr0` = 0.980 Kg
+ `arm_link_wr1` = 0.785 Kg
+ `arm_link_fngr` = 0.200 Kg 

## ROS 2 Control
The Spot URDF also has the option to be constructed with ROS 2 control tags, which uses the hardware interface plugin from the [`spot_hardware_interface`](../spot_hardware_interface) package. To get the plain URDF file containing these tags, run

```
ros2 run xacro xacro -o ./urdf/out/spot_ros2_control.urdf ./urdf/spot.urdf.xacro add_ros2_control_tag:=True
```
