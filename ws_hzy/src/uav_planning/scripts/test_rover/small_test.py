#! /usr/bin/env python

from read_yaml import get_yaml_data

if __name__ == "__main__":
    s="point"
    index=1
    res=get_yaml_data('/home/client1/catkin_ws_ros/src/uav_planning/scripts/test_rover/data.yaml')
    goal_x = res[s+str(index)]["x"] 
    goal_y = res[s+str(index)]["y"] 
    print(res)
    print(goal_x)
    print(goal_y)
    print("--------")	
    index+=1
    goal_x = res[s+str(index)]["x"] 
    goal_y = res[s+str(index)]["y"] 
    print(goal_x)
    print(goal_y)
    print("--------")	
