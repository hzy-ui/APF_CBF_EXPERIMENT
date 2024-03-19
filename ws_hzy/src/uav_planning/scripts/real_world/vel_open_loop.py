#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry


current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg



if __name__ == "__main__":
    rospy.init_node("test")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

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

    # goal point 
    goal_x = 0
    goal_y = 0
    goal_z = 2

    #Gain
    K1 = 1

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
        if rospy.get_time() - time_startup < 30:
                vel.twist.linear.x = 0
                vel.twist.linear.y = 0
                vel.twist.linear.z = 0.2
                rospy.loginfo("ACCENDING... " + str(rospy.get_time() - time_startup))
                

        else:
                vel.twist.linear.x =  0.0
                vel.twist.linear.y =  0.0
                vel.twist.linear.z =  0.0




        local_vel_pub.publish(vel)

        rate.sleep()

