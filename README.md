# frozone
Package to post process robot velocities to reduce the freezing robot problem.

## TODO:
1. Import jackal model with lidar
2. Test crowdmove in jackal simulation

## Dependencies:
* jackal_gazebo
* interactive_marker_twist_server
* robot_localization
* joint_state_controller
* diff_drive_controller
* CrowdMove mrca package and its dependencies
* Conda


Run these commands:
```
roslaunch jackal_gazebo jackal_world.launch
rosrun frozone goal_pub
```

Inside mrca package:
```
conda activate crowdmove
python final.py
```
