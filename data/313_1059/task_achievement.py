#!/usr/bin/python3

from cmath import cos, sin
import math
from numbers import Real
import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64
from uav_planning.msg import Barrier_info 
from nav_msgs.msg import Odometry
from potential_field import *
from CBF_control import CBF_controller_updates
from CBF_control import *
from Regulation import regulation_function
from Laser_data_process import *
import conn_uav as UAV
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from config import *

import math
NEVER_MIND = True

global flight_pose
flight_pose=PoseStamped()
last_pose_cb_time = None
def FLIGHT_pos_cb(msg_flight_pos):
    global last_pose_cb_time
    last_pose_cb_time = time.time()
    global flight_pose
    flight_pose.pose.position.x = msg_flight_pos.pose.position.x
    flight_pose.pose.position.y = msg_flight_pos.pose.position.y
    flight_pose.pose.position.z = msg_flight_pos.pose.position.z
    # TODO 注意航向角具体是多少
    flight_pose.pose.orientation.z = msg_flight_pos.pose.orientation.z
    # rospy.loginfo(f"get pose data{flight_pose.pose.position.x}, {flight_pose.pose.position.y}")


global laser_data
laser_data = LaserScan()
def LASER_data(msg_laser):
    global laser_data
    laser_data.ranges = msg_laser.ranges
    laser_data.angle_increment = msg_laser.angle_increment
    # 加了角度！！
    laser_data.angle_min = msg_laser.angle_min + flight_pose.pose.orientation.z
    # 滤波
    range_filtered = []
    for i in range(len(laser_data.ranges)):
        if laser_data.ranges[i] > 3:
            range_filtered.append(float('inf'))
        else:
            range_filtered.append(laser_data.ranges[i])
    # rospy.loginfo("get laser data")
    laser_data.ranges = tuple(range_filtered)
    laser_data_filtered = LaserScan()
    laser_data_filtered.header.frame_id = 'laser'
    laser_data_filtered.angle_min = laser_data.angle_min
    laser_data_filtered.angle_increment = laser_data.angle_increment
    laser_data_filtered.ranges = tuple(range_filtered)
    laser_data_filtered.intensities = laser_data.intensities
    laser_filtered_pub.publish(laser_data_filtered)


current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg


mission_state = WAITING
def order_cb(msg):
    global mission_state
    mission_state = msg.data
    print(mission_state)


def vel_ENU2FLU(v_E, v_N, v_U):
    yaw = float(flight_pose.pose.orientation.z.real).real
    v_E = v_E.real
    v_N = v_N.real
    v_U = v_U.real
    # 要求飞机刚体构建时机头朝东
    # 那么对于机体坐标系：
    #  v_F = v_E(v_x) * cos(yaw) + v_N(v_y) * sin(yaw)
    #  v_L =-v_E(v_x) * sin(yaw) + v_N(v_y) * cos(yaw)
    v_F = v_E * cos(yaw).real + v_N * sin(yaw).real
    v_L =-v_E * sin(yaw).real + v_N * cos(yaw).real
    return v_F, v_L, v_U

def set_uav_vel_global(v_x, v_y, v_z):
    v_F, v_L, v_U = vel_ENU2FLU(v_x, v_y, v_z)
    print(v_F, v_L, v_U)
    rospy.loginfo(f"LOCAL vf:{v_F} vl:{v_L}")
    UAV.uav_send_speed_FLU(v_F, v_L, 0)

# 定义圆柱体 Marker 的尺寸
CYLINDER_HEIGHT = 1.0
CYLINDER_RADIUS = 0.5

def draw_circle(obs_positions):
    """
    发布多个圆柱体 Markers.

    Args:
        positions_and_radiuses: 一个列表，包含每个圆柱体的位置和半径的三维元祖.
    """
    min_x, min_y, min_radius = 0,0,0
    min_dist = 100000
    for i, (x, y, radius) in enumerate(obs_positions):
        if (x - flight_pose.pose.position.x) ** 2 + (y - flight_pose.pose.position.y) ** 2 < min_dist:
            min_x = x
            min_y = y
            min_radius = radius
            min_dist = (x - flight_pose.pose.position.x) ** 2 + (y - flight_pose.pose.position.y) ** 2
    marker = Marker()
    marker.header.frame_id = "laser"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cylinders"
    marker.id = 1
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = min_x
    marker.pose.position.y = min_y
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 2.0 * min_radius
    marker.scale.y = 2.0 * min_radius
    marker.scale.z = CYLINDER_HEIGHT
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    markers_pub.publish(marker)


UAV_TAKEOFF_CLOSED_SUCCESS = 0
UAV_TAKEOFF_CLOSED_TIMEOUT = 1
UAV_TAKEOFF_CLOSED_SPEED   = 0.25
def uav_takeoff_closed(target_height = 1.0):
    for i in range(60):
        try:
            if check_vicon_state() != VICON_ONLINE:
                UAV.uav_hover()
                rospy.loginfo("[ERROR] While taking off, NO POSITION GET!")
            if flight_pose.pose.position.z >= target_height:
                rospy.loginfo("[STATE] UAV reached target height")
                UAV.uav_hover()
                return UAV_TAKEOFF_CLOSED_SUCCESS
            UAV.uav_send_speed_FLU(0,0,UAV_TAKEOFF_CLOSED_SPEED)
            time.sleep(0.1)
        except KeyboardInterrupt:
            UAV.uav_hover()
            rospy.loginfo("User kill the process - 1")
    return UAV_TAKEOFF_CLOSED_TIMEOUT


VICON_OFFLINE = 0
VICON_ONLINE  = 1
VICON_TIMEOUT = 1
def check_vicon_state():
    if last_pose_cb_time is not None:
        if last_pose_cb_time - time.time() > VICON_TIMEOUT:
            rospy.loginfo(f"[ERROR] NO POSITION FOR {VICON_TIMEOUT} SECOND!!!!!")
            return VICON_OFFLINE
        else:
            return VICON_ONLINE
    return VICON_OFFLINE
# TODO
# time_startup = 0
def gen_ctrl_uav(k, rho, attractive_gain, repulsive_gain, x_goal, t_star, t_pre):
    x = (flight_pose.pose.position.x, flight_pose.pose.position.y)
    try:
        obs_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min, flight_pose.pose.position.x, flight_pose.pose.position.y)
    except:
        rospy.loginfo("[ERROR] caculating obs")
        return
    t = rospy.get_time() - (t_pre + time_startup) 
    # print(t)
    # v1, v2, h, gamma, b, b_t= CBF_controller_updates(x, x_initial, x_goal, R, rho, k, t_star, t) # update
    v1, v2, h, gamma, b= CBF_controller(x, x_goal, R, rho, k, t_star, t) # no update
    # obs_positions = laser_data_process(laser_data.ranges,laser_data.angle_increment, laser_data.angle_min, flight_pose.pose.position.x, flight_pose.pose.position.y)
    
    # obs_positions = []
    # print("v1:"+ str(v1))
    # print("v2:"+ str(v2))
    # attractive_gain = 0
    # x_goal = [0, 5]
    if obs_positions != (float('inf'), 0, float('inf')) and obs_positions !=([]):
        total_force_x, total_force_y = potential_field_dynamic(flight_pose.pose.position.x, flight_pose.pose.position.y, obs_positions, x_goal, attractive_gain, repulsive_gain)
        u1, u2 = regulation_function(total_force_x, total_force_y, v1, v2, x, obs_positions)
        draw_circle(obs_positions)
        rospy.loginfo("obstacle detected " + str(obs_positions))
        rospy.loginfo(f"TARGET X:{x_goal[0]:6.3f}, Y:{x_goal[1]:6.3f}")
        rospy.loginfo(f"FORCE  X:{total_force_x:6.3}, Y:{total_force_y:6.3f}")
        rospy.loginfo(f"OUTPUT X:{u1:6.3}, Y:{u2:6.3f}")
    else:
        u1 = v1
        u2 = v2
        # rospy.loginfo("No obstacle detected" )
    # if math.sqrt(u1**2 + u2**2) >= 1: # uncommend if constraint is required
    #     u1 = math.sqrt(1) * ( u1/math.sqrt(u1**2 + u2**2) )
    #     u2 = math.sqrt(1) * ( u2/math.sqrt(u1**2 + u2**2) )
    # return
    vel.twist.linear.x = u1
    vel.twist.linear.y = u2
    vel.twist.linear.z = K1 *  (init_z - flight_pose.pose.position.z)
    # rospy.loginfo("the CBF value " + str(b))
    # rospy.loginfo("the h value " + str(h))
    # rospy.loginfo("the gamma value " + str(gamma))
    rospy.loginfo("complete the mission... " + str(rospy.get_time() - time_startup))
    rospy.loginfo(f"GLOBAL px:{x[0]:6.3f} py:{x[1]:6.3f}")
    rospy.loginfo(f"GLOBAL sx:{x_goal[0]:6.3f} sy:{x_goal[1]:6.3f}")
    rospy.loginfo(f"GLOBAL ux:{u1:6.3f} uy:{u2:6.3f}")

        # we need data
    data_info.t = rospy.get_time() - time_startup
    data_info.h = h
    data_info.gamma = gamma
    data_info.b = b
    data_info.u1 = u1
    data_info.u2 = u2
    # data_info.b_t = b_t
    data_info.x = flight_pose.pose.position.x
    data_info.y = flight_pose.pose.position.y
    
    return vel, data_info
            
            
if __name__ == "__main__":
    rospy.init_node("uav_planner")
    # 读取参数，如果没有提供参数则使用默认值
    uav_conn_protocol = rospy.get_param("~uav_conn_protocol", "uart")
    uav_conn_baundrate = rospy.get_param("~uav_conn_baundrate", UAV.DEFAULT_BAUNDRATE)
    uav_conn_serial_port = rospy.get_param("~uav_conn_serial_port", UAV.DEFAULT_PORT)
    UAV.init_uav_conn(uav_conn_serial_port, uav_conn_baundrate)
    # state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    # 指令消息接收
    order_sub = rospy.Subscriber("/mission_order", Int64, callback=order_cb)

    flight_pos_sub = rospy.Subscriber("/vicon/pose", PoseStamped, callback = FLIGHT_pos_cb)
    # flight_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = FLIGHT_pos_cb)

    # laser_data_sub = rospy.Subscriber("/laser/scan", LaserScan, callback = LASER_data)
    laser_data_sub = rospy.Subscriber("/scan", LaserScan, callback = LASER_data)

    local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    data_pub = rospy.Publisher("/barrier_information", Barrier_info, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    markers_pub = rospy.Publisher("/cylinders", Marker, queue_size=1)
    laser_filtered_pub = rospy.Publisher("/laser_filtered", LaserScan, queue_size=1)
    rospy.loginfo("等待连接飞控")
    
    rospy.loginfo("等待连接地面站")
    # TO DO 等待连接FCU
    # rospy.wait_for_service("/mavros/cmd/arming")
    # arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    # rospy.wait_for_service("/mavros/set_mode")
    # set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # TO DO 等待连接GCS

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    # while(not rospy.is_shutdown() and not current_state.connected):
    #     rate.sleep()

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
    repulsive_gain = 5.0  # Example repulsive gain
    repulsive_radius = 1.3  # Example repulsive radius

    # CBF parameters
    x_initial = (0, 0)
    x_goal = (-2, 3)  
    R = 0.5
    k = 1.5
    rho = 0.2
    t_star = 70 
    # v1, v2 = CBF_controller(x, x_inital, x_goal, R, rho, k, t_star, t)
    
    # while True:
    #     try:
    #         gen_ctrl_uav(k, rho, attractive_gain, repulsive_gain, x_goal, 40, 0)
    #         time.sleep(0.1)
    #     except KeyboardInterrupt:
    #         exit()
    


    # Gain accending
    K1 = 1
    # Potential field parameter 
    K_p = 0.1

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
    data_info.b_t = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_vel_pub.publish(vel)
        rate.sleep()

    # TODO
    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = 'OFFBOARD'

    # wating for vicon
    # rospy.wait_for_message("/vicon/pose")
    rospy.loginfo("[TASK INFO] GOT VICON POSE!")
    # arm_cmd = CommandBoolRequest()
    # arm_cmd.value = True
    # arm
    while not mission_state == ARM:
        try:
            rospy.loginfo("等待解锁指令")
            time.sleep(0.2)
        except KeyboardInterrupt:
            rospy.loginfo("用户结束进程-1")
            sys.exit()
    rospy.loginfo("正在解锁飞机...")
    UAV.arm_uav()
    time.sleep(1)
    UAV.arm_uav()
    time.sleep(1)
    while not mission_state == TAKEOFF:
        try:
            rospy.loginfo("等待起飞指令")
            rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("用户结束进程-2")
            sys.exit()
    UAV.uav_take_off()
    #uav_takeoff_closed()
    # 等待任务开始指令
    while not mission_state == TASK_BEGIN:
        rospy.loginfo("等待任务开始指令--"+str(TASK_BEGIN))
        time.sleep(0.5)
    last_req = rospy.Time.now()
    time_startup = rospy.get_time()

    while(not rospy.is_shutdown()):
        if mission_state == HOVER or mission_state == TASK_END:
            # stop the uav
            rospy.loginfo("Switch to #### LAND #### mode!")
            UAV.uav_hover()
            continue
            
        if mission_state == LAND:
            # land UAV
            rospy.loginfo("Switch to #### LAND #### mode!")
            UAV.uav_hover()
            rate.sleep()
            rospy.loginfo("Ready to #### LAND ####!")
            UAV.uav_land()
            rospy.loginfo("#### LAND END ####!")
            break
            
    #############################
        if rospy.get_time() - time_startup < 0:
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
            data_info.b_t = 0
            data_info.x = flight_pose.pose.position.x
            data_info.y = flight_pose.pose.position.y
        
        elif rospy.get_time() - time_startup > 0 and rospy.get_time() - time_startup < 20:
            k = 0.5
            rho = 0.02
            attractive_gain = 0.1 # Example attractive gain
            x_goal = (2, 3.2)  
            t_star = 15
            t_pre = 0
            gen_ctrl_uav(k, rho, attractive_gain, repulsive_gain, x_goal, t_star, t_pre)

        elif rospy.get_time() - time_startup >= 20 and rospy.get_time() - time_startup <= 60:
            k = 1
            rho = 0.02
            attractive_gain = 0.1 # Example attractive gain
            repulsive_gain = 4
            x_goal = (-2, 3.17)  
            t_star = 30
            t_pre = 20
            
            gen_ctrl_uav(k, rho, attractive_gain, repulsive_gain, x_goal, t_star, t_pre)
 
        elif rospy.get_time() - time_startup >= 60 and rospy.get_time() - time_startup < 120:
            k = 1
            rho = 0.02
            x_goal = (1, -3.5)  
            t_star = 55
            t_pre = 60
            
            attractive_gain = 0.1 # Example attractive gain
            repulsive_gain = 5
            gen_ctrl_uav(k, rho, attractive_gain, repulsive_gain, x_goal, t_star, t_pre)
        elif rospy.get_time() - time_startup >= 120:
            rospy.loginfo("UAV LAND")
            UAV.uav_land()
        else:
            UAV.uav_hover()

        local_vel_pub.publish(vel)
        data_pub.publish(data_info)
        # 速度映射 ENU --> FLU
        """
                N Y
                |
                |      
                |
             U Z·------------E Y
        v_F = 
        """
        # global last_pose_cb_time
        
        set_uav_vel_global(vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z)
        rate.sleep()

