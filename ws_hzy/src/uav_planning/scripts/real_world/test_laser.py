#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan

"""
    雷达坐标系
    线是屁股 180度，一帧共计360个点
    从机头开始
    id逆时针为正
          机头
          0
          
    90        270

         180
          线
    角度从min为-3.124（-pi） 开始一度一个点 inc=0.0174 到+pi
            -pi/pi
    
    
    -pi/2               pi/2

        
               0

"""
def laser_cb(msg):
    msg:LaserScan
    angle_min = msg.angle_min
    angle_inc = msg.angle_increment
    ranges = msg.ranges
    num_dot = len(ranges)
    id0 = 0
    id60    = int(num_dot / 6.)
    id120   = id60 * 2
    id180   = id60 * 3
    id240   = id60 * 4
    id300   = id60 * 5
    print("==="*20)
    print(f"共{num_dot}个点, min: {angle_min} inc: {angle_inc}")
    print(f"id60 :{id60}")
    print(f"0  度 -- {ranges[id0    ]}")
    print(f"60 度 -- {ranges[id60   ]}")
    print(f"120度 -- {ranges[id120  ]}")
    print(f"180度 -- {ranges[id180  ]}")
    print(f"240度 -- {ranges[id240  ]}")
    print(f"300度 -- {ranges[id300  ]}")
    
    

rospy.init_node("testlaser")

rospy.Subscriber("/scan", LaserScan, laser_cb)
rospy.spin()