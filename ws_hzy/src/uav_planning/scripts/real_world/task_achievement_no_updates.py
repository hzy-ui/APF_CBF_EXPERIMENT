#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import LaserScan
from uav_planning.msg import Barrier_info 
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from potential_field import *
from CBF_control import *
from Regulation import regulation_function
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

    data_pub = rospy.Publisher("/barrier_information", Barrier_info, queue_size=10)
   
    
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
    init_z = 2
    # goal point 
    goal_x = -6
    goal_y = -6

    obstacles = [(4, 4, 0.5)]  # Example obstacle positions
    x_goal = (goal_x, goal_y)  # Example goal position
    #robot_radius = 0.5  # Example robot radius
    attractive_gain = 0.1 # Example attractive gain
    repulsive_gain = 15.0  # Example repulsive gain
    repulsive_radius = 1.0  # Example repulsive radius

    # CBF parameters
    x_initial = (0, 0)
    x_goal = (goal_x, goal_y)  
    R = 1
    k = 1.5
    rho = 0.2
    # t_star = 70 
    # v1, v2 = CBF_controller(x, x_inital, x_goal, R, rho, k, t_star, t)


    # Gain accending
    K1 = 1
    # Potential field parameter 
    K_p = 0.05

    vel = TwistStamped()
    vel.twist.linear.x = 0
    vel.twist.linear.y = 0
    vel.twist.linear.z = 0


    data_info = Barrier_info()
    data_info.t = 0
    data_info.h = 0
    data_info.gamma = 0
    data_info.b = 0
    data_info.u1 = 0
    data_info.u2 = 0
    data_info.x = 0
    data_info.y = 0

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
        if rospy.get_time() - time_startup <= 20:
            vel.twist.linear.x = 0
            vel.twist.linear.y = 0
            vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
            rospy.loginfo("ACCENDING... " + str(rospy.get_time() - time_startup))
            # we need data
            data_info.t = rospy.get_time() - time_startup
            data_info.h = 0
            data_info.gamma = 0
            data_info.b = 0
            data_info.u1 = 0
            data_info.u2 = 0
            data_info.x = flight_pose.pose.position.x
            data_info.y = flight_pose.pose.position.y
        
        elif rospy.get_time() - time_startup > 20 and rospy.get_time() - time_startup < 40:
            k = 0.13
            rho = 0.02
            attractive_gain = 0.1 # Example attractive gain
            x_goal = (-6, -6)  
            t_star = 15
            x = (flight_pose.pose.position.x, flight_pose.pose.position.y)
            t = rospy.get_time() - (20 + time_startup) 
            # print(t)
            v1, v2, h, gamma, b= CBF_controller(x, x_goal, R, rho, k, t_star, t)
            obs_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min, flight_pose.pose.position.x, flight_pose.pose.position.y)

            if obs_positions != (float('inf'), 0, float('inf')) and obs_positions !=([]):
                total_force_x, total_force_y = potential_field_dynamic(flight_pose.pose.position.x, flight_pose.pose.position.y, obs_positions, x_goal, attractive_gain, repulsive_gain)
                u1, u2 = regulation_function(total_force_x, total_force_y, v1, v2, x, obs_positions)
                rospy.loginfo("obstacle detected " + str(obs_positions))
            else:
                u1 = v1
                u2 = v2
                rospy.loginfo("No obstacle detected" )
            
            # if math.sqrt(u1**2 + u2**2) >= 1:
            #     u1 = math.sqrt(1) * ( u1/math.sqrt(u1**2 + u2**2) )
            #     u2 = math.sqrt(1) * ( u2/math.sqrt(u1**2 + u2**2) )

            vel.twist.linear.x = u1
            vel.twist.linear.y = u2
            vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
            rospy.loginfo("the CBF value " + str(b))
            rospy.loginfo("the h value " + str(h))
            rospy.loginfo("the gamma value " + str(gamma))
            rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))

             # we need data
            data_info.t = rospy.get_time() - time_startup
            data_info.h = h
            data_info.gamma = gamma
            data_info.b = b
            data_info.u1 = u1
            data_info.u2 = u2
            data_info.x = flight_pose.pose.position.x
            data_info.y = flight_pose.pose.position.y

        elif rospy.get_time() - time_startup >= 40 and rospy.get_time() - time_startup <= 140:
            k = 1.5
            rho = 0.2
            x_goal = (6, -6)  
            t_star = 90
            x = (flight_pose.pose.position.x, flight_pose.pose.position.y)
            t = rospy.get_time() - (40 + time_startup) 
            # print(t)
            v1, v2, h, gamma, b= CBF_controller(x, x_goal, R, rho, k, t_star, t)
            obs_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min, flight_pose.pose.position.x, flight_pose.pose.position.y)
            # repulsive_gain = 100
            if obs_positions != (float('inf'), 0, float('inf')) and obs_positions !=([]):
                total_force_x, total_force_y = potential_field_dynamic(flight_pose.pose.position.x, flight_pose.pose.position.y, obs_positions, x_goal, attractive_gain, repulsive_gain)
                u1, u2 = regulation_function(total_force_x, total_force_y, v1, v2, x, obs_positions)
                rospy.loginfo("obstacle detected " + str(obs_positions))
            else:
                u1 = v1
                u2 = v2
                rospy.loginfo("No obstacle detected" )
            
            # if math.sqrt(u1**2 + u2**2) >= 1:
            #     u1 = math.sqrt(1) * ( u1/math.sqrt(u1**2 + u2**2) )
            #     u2 = math.sqrt(1) * ( u2/math.sqrt(u1**2 + u2**2) )

            vel.twist.linear.x = u1
            vel.twist.linear.y = u2
            vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
            rospy.loginfo("the CBF value " + str(b))
            rospy.loginfo("the h value " + str(h))
            rospy.loginfo("the gamma value " + str(gamma))
            rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))

            # we need data
            data_info.t = rospy.get_time() - time_startup
            data_info.h = h
            data_info.gamma = gamma
            data_info.b = b
            data_info.u1 = u1
            data_info.u2 = u2
            data_info.x = flight_pose.pose.position.x
            data_info.y = flight_pose.pose.position.y


        elif rospy.get_time() - time_startup >= 140 and rospy.get_time() - time_startup < 240:
            k = 0.15
            rho = 0.02

            x_goal = (-6, 6)  
            t_star = 90
            x = (flight_pose.pose.position.x, flight_pose.pose.position.y)
            t = rospy.get_time() - (140 + time_startup) 
            # print(t)
            v1, v2, h, gamma, b= CBF_controller(x, x_goal, R, rho, k, t_star, t)
            obs_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min, flight_pose.pose.position.x, flight_pose.pose.position.y)
            repulsive_gain = 60
            if obs_positions != (float('inf'), 0, float('inf')) and obs_positions !=([]):
                total_force_x, total_force_y = potential_field_dynamic(flight_pose.pose.position.x, flight_pose.pose.position.y, obs_positions, x_goal, attractive_gain, repulsive_gain)
                u1, u2 = regulation_function(total_force_x, total_force_y, v1, v2, x, obs_positions)
                rospy.loginfo("obstacle detected " + str(obs_positions))
            else:
                u1 = v1
                u2 = v2
                rospy.loginfo("No obstacle detected" )
            
            # if math.sqrt(u1**2 + u2**2) >= 1:
            #     u1 = math.sqrt(1) * ( u1/math.sqrt(u1**2 + u2**2) )
            #     u2 = math.sqrt(1) * ( u2/math.sqrt(u1**2 + u2**2) )

            vel.twist.linear.x = u1
            vel.twist.linear.y = u2
            vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
            rospy.loginfo("the CBF value " + str(b))
            rospy.loginfo("the h value " + str(h))
            rospy.loginfo("the gamma value " + str(gamma))
            rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))

            # we need data
            data_info.t = rospy.get_time() - time_startup
            data_info.h = h
            data_info.gamma = gamma
            data_info.b = b
            data_info.u1 = u1
            data_info.u2 = u2
            data_info.x = flight_pose.pose.position.x
            data_info.y = flight_pose.pose.position.y
        else:
            # vel.twist.linear.x =  0.1 *  (init_x - flight_pose.pose.position.x)
            # vel.twist.linear.y =  0.1 *  (init_y - flight_pose.pose.position.y)
            # vel.twist.linear.z =  K1 *  (init_z - flight_pose.pose.position.z)
            # rospy.loginfo("homing... " + str(rospy.get_time() - time_startup))
            k = 0.15
            rho = 0.02
            x_goal = (0, 0)  
            t_star = 100
            x = (flight_pose.pose.position.x, flight_pose.pose.position.y)
            t = rospy.get_time() - (300 + time_startup) 
            v1, v2, h, gamma, b= CBF_controller(x, x_goal, R, rho, k, t_star, t)
            obs_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min, flight_pose.pose.position.x, flight_pose.pose.position.y)
            repulsive_gain = 80
            if obs_positions != (float('inf'), 0, float('inf')) and obs_positions !=([]):
                total_force_x, total_force_y = potential_field_dynamic(flight_pose.pose.position.x, flight_pose.pose.position.y, obs_positions, x_goal, attractive_gain, repulsive_gain)
                u1, u2 = regulation_function(total_force_x, total_force_y, v1, v2, x, obs_positions)
                rospy.loginfo("obstacle detected " + str(obs_positions))
            else:
                u1 = v1
                u2 = v2
                rospy.loginfo("No obstacle detected" )
            
            # if math.sqrt(u1**2 + u2**2) >= 1:
            #     u1 = math.sqrt(1) * ( u1/math.sqrt(u1**2 + u2**2) )
            #     u2 = math.sqrt(1) * ( u2/math.sqrt(u1**2 + u2**2) )


            vel.twist.linear.x = u1
            vel.twist.linear.y = u2
            vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
            rospy.loginfo("the CBF value " + str(b))
            rospy.loginfo("the h value " + str(h))
            rospy.loginfo("the gamma value " + str(gamma))
            rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))

            # we need data
            data_info.t = rospy.get_time() - time_startup
            data_info.h = h
            data_info.gamma = gamma
            data_info.b = b
            data_info.u1 = u1
            data_info.u2 = u2
            data_info.x = flight_pose.pose.position.x
            data_info.y = flight_pose.pose.position.y


        local_vel_pub.publish(vel)
        data_pub.publish(data_info)


        rate.sleep()

