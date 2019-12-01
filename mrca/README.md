# CrowdMove

---

## Environment
* Ubuntu 16.04
* ROS Kinetic
* python==2.7
* tensorflow >= 1.4
* tensorlayer

---

## Install
1. install Pozyx UWB
``` shell
cd Pozyx-Python-library
pip install -e .
```

2. setup UWB id in /Pozyx-Python-library/tutorials/turtlebot_follow.py
``` python
# the tag on the target person
tags = [0x6042]
# the anchors on the turtlebot
anchors = [DeviceCoordinates(0x676a, 1, Coordinates(-165, 0, 405)),
            DeviceCoordinates(0x6e23, 1, Coordinates(175, 0, 405)),
            DeviceCoordinates(0x6e63, 1, Coordinates(-5, 125, 405)),
            DeviceCoordinates(0x6e1f, 1, Coordinates(5, -115, 405))]
```

---

## Run

1. start turtlebot
``` shell
roslaunch turtlebot_bringup minimal.launch
```

2. start hokuyo lidar
``` shell
rosrun hokuyo_node hokuyo_node
```

3. start UWB localization system
``` shell
cd Pozyx-Python-library/tutorials/
python turtlebot_follow.py
```

4. start crowdmove
``` shell
python agent_follow_me.py
```
