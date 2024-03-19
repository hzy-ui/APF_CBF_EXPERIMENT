#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from read_yaml import get_yaml_data

#global flight_pose
#global flight_vel
#global turtle_pose
current_state = State()
flight_pose=PoseStamped()
flight_vel=TwistStamped()
turtle_pose=Odometry()

def state_cb(msg):
    global current_state
    current_state = msg

def FLIGHT_pos(msg_flight_pos):
    global flight_pose
    flight_pose.pose.position.x = msg_flight_pos.pose.position.x
    flight_pose.pose.position.y = msg_flight_pos.pose.position.y
    flight_pose.pose.position.z = msg_flight_pos.pose.position.z

'''
def FLIGHT_vel(msg_flight_vel):
    global flight_vel
    flight_vel.twist.linear.x = msg_flight_vel.twist.linear.x
    flight_vel.twist.linear.y = msg_flight_vel.twist.linear.y
    flight_vel.twist.linear.z = msg_flight_vel.twist.linear.z
'''

def turtlebot_pos(msg_turtle_pos):
    global turtle_pose
    turtle_pose.pose.pose.position.x = msg_turtle_pos.pose.pose.position.x
    turtle_pose.pose.pose.position.y = msg_turtle_pos.pose.pose.position.y


if __name__ == "__main__":
    #global flight_pose
    #global turtle_pose
    #global flight_vel
    rospy.init_node("test")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    flight_pos_sub = rospy.Subscriber("/uav_2/uwb_pose/pose", PoseStamped, callback = FLIGHT_pos)
  #  flight_vel_sub = rospy.Subscriber("/uav_2/uwb_pose/vel", TwistStamped, callback = FLIGHT_vel)

    t_state_sub = rospy.Subscriber("odom", Odometry, callback = turtlebot_pos)
    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    for i in range(0, 60):
        rate.sleep()
    # Wait for Flight Controller connection
    #while(not rospy.is_shutdown() and not current_state.connected):
    #    rate.sleep()

    vel = TwistStamped()

 #    vel.twist.linear.x = 0
 #   vel.twist.linear.y = 0
 #   vel.twist.linear.z = 0

    # Send a few setpoints before starting
 #   for i in range(100):   
#       if(rospy.is_shutdown()):
 #           break

 #       local_vel_pub.publish(vel)
 #       rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    res=get_yaml_data('/home/client2/catkin_ws_ros/src/uav_planning/scripts/test_rover/data.yaml')
    #Gain
    K1 = 0.2
    K11 = 0.4 #gain for track 2D point 
    K2 = 0.00
    K22=0.00 #gain for track 2D point 
    # startup time
    rospy.sleep(1)

    # point index
    s="point"
    index=1
    is_armed = False
    last_req_arm = rospy.Time.now()    
    while not rospy.is_shutdown() and not is_armed:
        if rospy.Time.now() - last_req_arm > rospy.Duration(1.0):
            #is_armed = arming_client.call(arm_cmd).success
            arming_client.call(arm_cmd)
            is_armed = True

            rospy.loginfo("Arming vehicle...")                       #print(type(arming_client.call(arm_cmd)))

            last_req_arm = rospy.Time.now()
        
        rate.sleep()

    rospy.loginfo("Vehicle armed!")
	
    time_startup = rospy.get_time()
    last_time = rospy.get_time()

    goal_x = res["point_home"]["x"]
    goal_y = res["point_home"]["y"]
    goal_z = 1
    error_last_x = 0.0
    error_last_y = 0.0
    error_last_z = 0.0      

    print(goal_x)

   #     vel = TwistStamped()
        # goal point 
    while(not rospy.is_shutdown()):

        if rospy.get_time() - time_startup < 15:
                now_time = rospy.get_time() 
                d_t= now_time - last_time
                vel.twist.linear.x = K11 *  (goal_x - flight_pose.pose.position.x) + K22 * (goal_x - flight_pose.pose.position.x - error_last_x)/d_t
                vel.twist.linear.y = K11 *  (goal_y - flight_pose.pose.position.y) + K22 * (goal_y - flight_pose.pose.position.y - error_last_y)/d_t
                vel.twist.linear.z = K1 *  (goal_z - flight_pose.pose.position.z) + K22 * (goal_z - flight_pose.pose.position.z - error_last_z)/d_t
                error_last_x = goal_x - flight_pose.pose.position.x
                error_last_y = goal_y - flight_pose.pose.position.y
                error_last_z = goal_z - flight_pose.pose.position.z

                if index % 15 == 0:
                    rospy.loginfo("[Control Node] ACCENDING... \t" + "Z = " + str(goal_z) + "\t " + str(rospy.get_time() - time_startup))
                last_time = rospy.get_time() 
                
        elif rospy.get_time() - time_startup < res["point" + str(res.__len__() - 1)]["t2"]:
            for i in range(1, res.__len__()):
                if rospy.get_time() - time_startup >= res["point"+str(i)]["t1"] and rospy.get_time() - time_startup < res["point"+str(i)]["t2"]:
                    now_time = rospy.get_time() 
                    d_t= now_time - last_time
                    vel.twist.linear.x = K11 *  (res["point"+str(i)]["x"]  - flight_pose.pose.position.x)+ K22 * (res["point"+str(i)]["x"]  - flight_pose.pose.position.x - error_last_x)/d_t
                    vel.twist.linear.y = K11 *  (res["point"+str(i)]["y"]  - flight_pose.pose.position.y)+ K22 * (res["point"+str(i)]["y"]  - flight_pose.pose.position.y - error_last_y)/d_t
                    vel.twist.linear.z = K1 *  (goal_z - flight_pose.pose.position.z)+ K2 * (goal_z - flight_pose.pose.position.z - error_last_z)/d_t
                    error_last_x = res["point"+str(i)]["x"] - flight_pose.pose.position.x
                    error_last_y = res["point"+str(i)]["y"] - flight_pose.pose.position.y
                    error_last_z = goal_z - flight_pose.pose.position.z

                    if index % 15 == 0:
                        rospy.loginfo("[Control Node] DETECTING... " + str(rospy.get_time() - time_startup) + "\t Waypt:" + "point"+str(i) + " " + str(res["point"+str(i)]["x"]) + ", " + str(res["point"+str(i)]["y"]))
                    last_time = rospy.get_time()
        else:
                now_time = rospy.get_time() 
                d_t= now_time - last_time
                vel.twist.linear.x = K11 *  (res["point_home"]["x"]  - flight_pose.pose.position.x)
                vel.twist.linear.y = K11 *  (res["point_home"]["y"]  - flight_pose.pose.position.y)
 #               vel.twist.linear.z = K1 *  (0 - flight_pose.pose.position.z)
                vel.twist.linear.z = -0.1
                error_last_x = res["point_home"]["x"] - flight_pose.pose.position.x
                error_last_y = res["point_home"]["y"] - flight_pose.pose.position.y
                error_last_z = 0 - flight_pose.pose.position.z

                if index % 15 == 0:
                    rospy.loginfo("[Control Node] DECEDING... " + str(rospy.get_time() - time_startup))
                last_time = rospy.get_time() 

        # ENU -> NWU
        
        temp = vel.twist.linear.x
        vel.twist.linear.x = vel.twist.linear.y
        vel.twist.linear.y = temp				# -temp

	'''
	vel.twist.linear.x=vel.twist.linear.x
	vel.twist.linear.y=-vel.twist.linear.y
	'''
        if index % 45 == 0:
            rospy.loginfo("[Control Node] UAV Position: " + str(flight_pose.pose.position.x) + "\t" + str(flight_pose.pose.position.y) + "\t" + str(flight_pose.pose.position.z))

        local_vel_pub.publish(vel)

        index += 1
        if index > 1000:
            index = 0

        rate.sleep()

