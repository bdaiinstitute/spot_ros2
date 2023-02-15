The spot and its arm inertial properties are extracted from 'Isaac Sim', which based on the collision geometry and uniform mass distribution, defines the mass, CoM, and inertia tensor around the CoM frame. The inertia tensor is around its principal axes. Therefore the CoM frames are all rotated. 

For the arm, we use the following mass values
arm_link_sh0 + arm_link_sh1 = 2.596 Kg (sh0 : 90%, sh1 : 10%)
arm_link_hr0 = 0.0
arm_link_el0 + arm_link_el1 = 1.450 Kg (el0 : 50%, el1 : 50%)
arm_link_wr0 = 0.980 Kg
arm_link_wr1 = 0.785 Kg
arm_link_fngr = 0.200 Kg 

To get these inertial properties, use the following method:

    BdaiSim.import_inertia_tensor = False

    sh_mass = 2.596  # shoulder mass
    fa_mass = 1.450  # Forearm mass
    desired_mass = {"arm_link_sh0" :  0.9 * sh_mass, "arm_link_sh1" : 0.1 * sh_mass, 
                    "arm_link_hr0" : 1e-6,
                    "arm_link_el0" : 0.5 * fa_mass, "arm_link_el1" : 0.5 * fa_mass,
                    "arm_link_wr0" : 0.980, "arm_link_wr1" : 0.785,
                    "arm_link_fngr" : 0.200}
    BdaiSim.print_body_urdf(use_diagonal_inertia=True, desired_mass=desired_mass)
