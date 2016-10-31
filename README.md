# Dual Manipulation Grasp Database

This package contains the grasp database(s) used for [DualManipulation](https://bitbucket.org/dualmanipulation/dualmanipulation), as well as various utilities which can be used to make from scratch, include, or modify existing grasp trajectories.

The databases come in SQL format, and can be viewed using `sqlitebrowser` (or any other similar tool).

Grasp trajectories are stored in binary file form inside this package, under the name `grasp_trajectories/object($O_ID)/grasp($G_ID)`, where `O_ID` and `G_ID` are the link to the corresponding database entry; NOTICE THAT it is a user responsibility to make sure that the correspondence is respected.

One important piece of software included in this package is the `Grasp Modification Utility`, or simply `GMU`, which is described more in detail next.

## GMU: Grasp Modification Utility

Use the command `roslaunch dual_manipulation_grasp_db grasp_modification_utility.launch` for launching the `GMU` with standard arguments.
The arguments are:

- `robot_name`: the name of the robot for which to load the database; this will affect the parameters loaded by `dual_manipulation_shared` launch file `dual_manipulation.launch`
- `load_ik_params`: setting this value to true is only needed when running a real or simulated experiment, so it is usually not needed when modifying a grasp trajectory

This command will launch `rviz` with a custom panel that allows to load an already present grasp, or to create a new one.
Possible modifications involve:

- adding or removing waypoints
- modifying each of the waypoint (object/end-effector relative pose, actuated joint value(s), time from start)
- modifying the post-grasp pose (the pose close to the yellow object), which is used in the [DualManipulation](https://bitbucket.org/dualmanipulation/dualmanipulation) framework if object tracking during grasp is not active

After modifications have been made, the grasp (or its modifications) can either be discarded or saved.

It is possible to change the end-effector which is used in the utility: to this end, the following changes have to be made:

- in `launch/grasp_modification_utility.launch` file, change the parameters `hand_mesh_path`, `ee_link_name`, `actuated_joints`, `joints_lower_limit`, and `joints_upper_limit`
- in `grasp_modification_utility/urdf/hand.urdf.xacro` file, include the robot model of the end-effector you want to use, and make sure it is attached to the link named `hand`

## How to generate a new grasp from a PhaseSpace acquisition?

1. Create the OBJECT entry in the database, if it isn't already there (use `sqlitebrowser` for this purpose, modifying `object id`, `mesh resources`, etc )

2. Extract data from bag files `roslaunch dual_manipulation_grasp_db serializeGrasps.launch object:=OBJECT_NAME` (this step requires the package [unipi-grasp-dataset](https://github.com/CentroEPiaggio/unipi-grasp-datasets), follow the instructions in how to play the bagfiles in the [README](https://github.com/CentroEPiaggio/unipi-grasp-datasets/blob/master/pacman_wp2_db/README.md) )

3. Validate the recorded grasps
	- Inspect visually: `roslaunch dual_manipulation_grasp_db grasp_modification_utility.launch`
	- Inspect with the robot: run the demo with the fake planner to test a specific grasp id.
