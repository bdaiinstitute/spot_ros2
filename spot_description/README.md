The spot and its arm inertial properties are extracted from `Isaac Sim`, which based on the collision geometry and uniform mass distribution, defines the mass, CoM, and inertia tensor around the CoM frame. The inertia tensors are around their principal axes. Therefore the CoM frames are rotated. 

For the arm, we use the following mass values:
`arm_link_sh0` + `arm_link_sh1` = 2.596 Kg (sh0 : 90%, sh1 : 10%)
`arm_link_hr0` = 0.0
`arm_link_el0` + `arm_link_el1` = 1.450 Kg (el0 : 50%, el1 : 50%)
`arm_link_wr0` = 0.980 Kg
`arm_link_wr1` = 0.785 Kg
`arm_link_fngr` = 0.200 Kg 

To get these inertial properties, use the following method:

```
BdaiSim.import_inertia_tensor = False
sh_mass = 2.596  # shoulder mass
fa_mass = 1.450  # forearm mass
desired_mass = {"arm_link_sh0" :  0.9 * sh_mass, "arm_link_sh1" : 0.1 * sh_mass, 
                "arm_link_hr0" : 1e-6,
                "arm_link_el0" : 0.5 * fa_mass, "arm_link_el1" : 0.5 * fa_mass,
                "arm_link_wr0" : 0.980, "arm_link_wr1" : 0.785,
                "arm_link_fngr" : 0.200}
BdaiSim.print_body_urdf(use_diagonal_inertia=True, desired_mass=desired_mass)
```

Currently, this repository contains two spot models, referred to as `spot` and `spot simple`. The difference is mainly in the number of links in the base frame.

To get the plain URDF files, use the following commands in the package root:

```
# for simple spot
rosrun xacro xacro -o ./urdf/out/spot.urdf ./urdf/spot_simple.urdf.xacro

# for simple spot_with_arm
rosrun xacro xacro -o ./urdf/out/spot_with_arm.urdf.urdf ./urdf/spot_with_arm.urdf.urdf.xacro
```

To visualize and inspect the joint range, use the following command:

```
roslaunch spot_description urdf_viz.launch
```
