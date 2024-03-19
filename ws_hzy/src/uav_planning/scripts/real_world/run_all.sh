sudo chmod 777 /dev/ttyUSB*
sudo chmod +x /home/client3/ws_hzy/src/uav_planning/scripts/real_world/*.py
source ~/ws_hzy/devel/setup.bash 
roslaunch uav_planning task_achievement_real.launch
