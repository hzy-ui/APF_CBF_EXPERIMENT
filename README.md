# APF_CBF_EXPERIMENT
These are the experiment codes of implementation of the APF CBF control framework in a UAV within the VICON system to complete the allocated STL task.
The Experiment depends on the ROS (Melodic) environment which runs in Linux (Ubuntu 18.04).

## The Experiment configuration 
<!-- ![This is an alt text.](/image/lab_env_xyz.jpg "The VICON system configuration.")-->
The VICON system configuration:

<img src="/image/lab_env_xyz.jpg" alt="This is an alt text." style="width:500px;height:300px;">

The VICON coordinates: ENU; UAV local coordinates: FLU.

The aircraft configuration includes the following components: Ano P2 flight controller, a quadcopter design equipped with TMOTOR 2216 brushless motors and T1045 propellers. Onboard, there is an RPLidar single-line rotating LiDAR responsible for environmental sensing, while an Intel NUC serves as the onboard computer for executing algorithms and controlling UAV motion. The UAV receives global positioning updates from VICON via a data telemetry radio.

Flight controller serial port: /dev/ttyUSB0; Data telemetry radio: /dev/ttyUSB1; LiDAR: /dev/ttyUSB2.

First, check and enable the serial ports
```
ls /dev/ttyUSB
```
```
sudo chmod 777 /dev/ttyUSB*
```

## Launch
First, open a terminal and run:
```
roslaunch uav_planning task_achievement_real.launch
```
Then, in the file path "\ws_hzy\src\uav_planning\scripts\real_world", open a new terminal and run:
```
python3 uav_control_ros.py
```
Then, enter the command to unlock the UAV, take off, and execute the STL mission.

## Data
"2024-03-12-14-46-24.bag" is the correct data.

### Video
<!-- [Experiment Video](https://vimeo.com/manage/videos/924906196).
<video width="320" height="240" controls>
  <source src="[/video/experiment video.mp4](https://github.com/hzy-ui/APF_CBF_EXPERIMENT/raw/main/video/experiment%20video.mp4)" type="video/mp4">
  Your browser does not support the video tag.
</video>-->
<video src="https://github.com/HakunaMatata052/video/blob/master/a.mp4?raw=true" controls="controls">
您的浏览器不支持 video 标签。
</video>

