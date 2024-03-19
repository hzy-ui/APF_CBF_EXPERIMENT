# APF_CBF_VICON
These are the experiment codes of implementation of the APF CBF control framework in a UAV within the VICON system.
The Experiment depends on the ROS (Melodic) environment which runs in Linux (Ubuntu 18.04).

## The Experiment configuration 
![This is an alt text.](/image/sample.png "This is a sample image.")

## Blocks of code
First, open a terminal and run:
```
roslaunch uav_planning task_achievement_real.launch
```
Then, in the file path "\ws_hzy\src\uav_planning\scripts\real_world", open a new terminal and run:
```
python3 uav_control_ros.py
```
Then, enter the command to unlock the UAV, take off, and execute the STL mission.
