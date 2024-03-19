#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
import threading
 
global turtle_pose
current_state = State()
turtle_pose=Odometry()

def state_cb(msg):
    global current_state
    current_state = msg

def thread_job():
    rospy.spin()

def turtlebot_pos(msg_turtle_pos):
    global turtle_pose
    turtle_pose.pose.pose.position.x = msg_turtle_pos.pose.pose.position.x
    turtle_pose.pose.pose.position.y = msg_turtle_pos.pose.pose.position.y



if __name__ == "__main__":
    global turtle_pose
    rospy.init_node("test")
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    t_state_sub = rospy.Subscriber("odom", Odometry, callback = turtlebot_pos)
    rate.sleep()
  
    #add_thread = threading.Thread(target = thread_job)
    #add_thread.start()

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    #rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    rate.sleep()
    pose.pose.position.x = turtle_pose.pose.pose.position.x 
    pose.pose.position.y = turtle_pose.pose.pose.position.y 
    pose.pose.position.z = 2
 
    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
 

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
        rate.sleep()
        pose.pose.position.x = turtle_pose.pose.pose.position.x 
        pose.pose.position.y = turtle_pose.pose.pose.position.y 
        pose.pose.position.z = 2
        local_pos_pub.publish(pose)

        rate.sleep()



