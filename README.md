# Dual Manipulation Grasp Database

## How to generate a database ?

1. Create the OBJECT databse: `cp empty.db OBJECT_NAME.db`

2. `sqlitebrowser` to modify what is to be modified (object id, mesh resources, etc )

3. Extract data from bag files `roslaunch dual_manipulation_grasp_db serializeGrasps.launch object:=OBJECT_NAME` (this step requires the package [unipi-grasp-dataset](https://github.com/CentroEPiaggio/unipi-grasp-datasets), follow the instructions in how to play the bagfiles in the [README](https://github.com/CentroEPiaggio/unipi-grasp-datasets/blob/master/pacman_wp2_db/README.md) )

4. Validate the recorded grasps
	- Inspect visually: `roslaunch dual_manipulation_grasp_db grasp_modification_utility.launch`
	- Inspect with the robot: run the demo with the fake planner to test a specific grasp id.

5. Bootstrap the database with a db_app (create table and specular grasps, and grasp transitions)

6. Validate the bootstrapped grasps
	- Inspect visually: `roslaunch dual_manipulation_grasp_db grasp_modification_utility.launch`
	- Inspect with the robot: run the demo with the fake planner to test a specific grasp id.

7. IF another object? GOTO 1, ELSE create the full database `roslaunch dual_manipulation_grasp_db createFullDatabase.launch`

8. #sigh
