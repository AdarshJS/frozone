# frozone
Package to post process robot velocities to reduce the freezing robot problem.

## TODO:
1. Import jackal model with lidar
2. Test crowdmove in jackal simulation

## Dependencies:
* jackal_gazebo
* CrowdMove mrca package and its dependencies
* Conda


Run these commands:
1. roslaunch jackal_gazebo jackal_world.launch
2. rosrun frozone goal_pub

Inside mrca package:
3. conda activate crowdmove
4. python final.py
