#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from potential_field import potential_field



global flight_pose
flight_pose=PoseStamped()
def FLIGHT_pos(msg_flight_pos):
    global flight_pose
    flight_pose.pose.position.x = msg_flight_pos.pose.position.x
    flight_pose.pose.position.y = msg_flight_pos.pose.position.y
    flight_pose.pose.position.z = msg_flight_pos.pose.position.z


current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg



if __name__ == "__main__":
    rospy.init_node("test")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    flight_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = FLIGHT_pos)
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
    init_z = 2
    # goal point 
    goal_x = 5
    goal_y = 5

    obstacles = [(2.5, 2.5, 1)]  # Example obstacle positions
    goal = (goal_x, goal_y)  # Example goal position
    #robot_radius = 0.5  # Example robot radius
    attractive_gain = 0.1 # Example attractive gain
    repulsive_gain = 10.0  # Example repulsive gain
    repulsive_radius = 2.0  # Example repulsive radius


    # Gain accending
    K1 = 1
    # Potential field parameter 
    K_p =0.1

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
        if rospy.get_time() - time_startup < 20:
                vel.twist.linear.x = 0
                vel.twist.linear.y = 0
                vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
                rospy.loginfo("ACCENDING... " + str(rospy.get_time() - time_startup))
        
        elif rospy.get_time() - time_startup >= 20 and rospy.get_time() - time_startup < 60:
                total_force_x, total_force_y = potential_field(flight_pose.pose.position.x, flight_pose.pose.position.y, obstacles, goal, attractive_gain, repulsive_gain, repulsive_radius)
                vel.twist.linear.x = total_force_x
                vel.twist.linear.y = total_force_y
                vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
                rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))
        else:
                vel.twist.linear.x =  0.1 *  (init_x - flight_pose.pose.position.x)
                vel.twist.linear.y =  0.1 *  (init_y - flight_pose.pose.position.y)
                vel.twist.linear.z =  K1 *  (init_z - flight_pose.pose.position.z)
                rospy.loginfo("homing... " + str(rospy.get_time() - time_startup))




        local_vel_pub.publish(vel)

        rate.sleep()

