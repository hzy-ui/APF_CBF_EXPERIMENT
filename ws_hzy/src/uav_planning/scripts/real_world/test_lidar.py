#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from Laser_data_process import *


global flight_pose
flight_pose=PoseStamped()
def FLIGHT_pos(msg_flight_pos):
    global flight_pose
    flight_pose.pose.position.x = msg_flight_pos.pose.position.x
    flight_pose.pose.position.y = msg_flight_pos.pose.position.y
    flight_pose.pose.position.z = msg_flight_pos.pose.position.z



global laser_data
laser_data = LaserScan()
def LASER_data(msg_laser):
    global laser_data
    laser_data.ranges = msg_laser.ranges
    laser_data.angle_increment = msg_laser.angle_increment
    laser_data.angle_min = msg_laser.angle_min


current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg



if __name__ == "__main__":
    rospy.init_node("test")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    flight_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = FLIGHT_pos)

    laser_data_sub = rospy.Subscriber("/laser/scan", LaserScan, callback = LASER_data)

    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
   
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # initial point 
    init_x = 0
    init_y = 0
    init_z = 0.4
    # goal point 
    goal_x = 5
    goal_y = 5

    # Gain accending
    K1 = 1
    # Potential field parameter 
    K_p = 0.1

    vel = TwistStamped()
    vel.twist.linear.x = 0
    vel.twist.linear.y = 0
    vel.twist.linear.z = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_vel_pub.publish(vel)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    time_startup = rospy.get_time()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
                
	#############################
        if rospy.get_time() - time_startup < 2000:
                vel.twist.linear.x = 0
                vel.twist.linear.y = 0
                vel.twist.linear.z = 0.5
                rospy.loginfo("ACCENDING... " + str(rospy.get_time() - time_startup))
                # print(laser_data.ranges)
                relative_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min,flight_pose.pose.position.x,flight_pose.pose.position.y)
                print(relative_positions)
        
        elif rospy.get_time() - time_startup >= 2000 and rospy.get_time() - time_startup < 4000:
                vel.twist.linear.x = K_p *  (goal_x - flight_pose.pose.position.x)
                vel.twist.linear.y = K_p *  (goal_y - flight_pose.pose.position.y)
                vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
                rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))
        else:
                vel.twist.linear.x =  0.1 *  (init_x - flight_pose.pose.position.x)
                vel.twist.linear.y =  0.1 *  (init_y - flight_pose.pose.position.y)
                vel.twist.linear.z =  K1 *  (init_z - flight_pose.pose.position.z)
                rospy.loginfo("homing... " + str(rospy.get_time() - time_startup))




        local_vel_pub.publish(vel)

        rate.sleep()

