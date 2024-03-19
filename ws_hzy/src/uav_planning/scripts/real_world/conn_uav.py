#!/usr/bin/python3
"""
坐标系：位置坐标 VICON ENU --> 控制量UAV FLU
"""
import time
from pymavlink import mavutil
import serial
import warnings
# mavutil.set_dialect("common")
import os
# os.environ['MAVLINK20'] = '1'
# mavutil.set
# try:
# fcu_ser = Serial("/dev/ttyUSB0", baudrate=500000)
# fcu_master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=500000,source_system=1, source_component=1)

# 参数初始化
ROS_ON = True
# ROS_ON = False
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUNDRATE = 500000
if ROS_ON:
    import rospy
    
    
def init_uav_conn(uav_conn_serial_port, uav_conn_baundrate):
    print(f"读取到了与飞机连接的参数:port:{uav_conn_serial_port}@{uav_conn_baundrate}")
    # 不适用ros
    print(DEFAULT_PORT, DEFAULT_BAUNDRATE)
    global fcu_master
    fcu_master = mavutil.mavlink_connection(uav_conn_serial_port, baud=uav_conn_baundrate)
    fcu_master.mav.version = 'v2.0'
# try:
#     gcs_ser = serial.Serial(uav_conn_serial_port, baudrate=uav_conn_baundrate)
#     gcs_master = mavutil.mavlink_connection(gcs_ser,source_system=255, source_component=1)
#     gcs_master.mavlink.version = 'v2.0'
# except Exception:
#     warnings.warn("打开串口出现错误，无法连接飞控或地面站，请检查串口顺序，并使能串口")
    
def uav_send_speed_ned(spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec):
    type_mask = 0b0000111111000111  # 忽略位置信息，只设置速度
    fcu_master.mav.set_position_target_local_ned_send(
        0,  # 时间戳，0 表示立即执行
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 使用局部NED坐标系
        type_mask,
        0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
        spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec,  # 目标速度
        0, 0, 0,  # 目标加速度（忽略）
        0, 0  # 目标偏航角度和偏航速率（忽略）
    )
    
def uav_send_speed_FLU(spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec):
    type_mask = 0b0000111111000111  # 忽略位置信息，只设置速度
    if spd_x_m_per_sec > 1.0:
        spd_x_m_per_sec = 1.0
    if spd_x_m_per_sec < -1.0:
        spd_x_m_per_sec = -1.0
    if spd_y_m_per_sec > 1.0:
        spd_y_m_per_sec = 1.0
    if spd_y_m_per_sec < -1.0:
        spd_y_m_per_sec = -1.0
    fcu_master.mav.set_position_target_local_ned_send(
        0,  # 时间戳，0 表示立即执行
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
        type_mask,
        0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
        spd_x_m_per_sec, spd_y_m_per_sec, spd_z_m_per_sec,  # 目标速度
        0, 0, 0,  # 目标加速度（忽略）
        0, 0  # 目标偏航角度和偏航速率（忽略）
    )

def uav_turn_yaw_rad(rad):
    # 北零 左逆负，右顺正 -pi -- pi
    type_mask = 0b0000101111111111  # 忽略位置信息，只设置速度
    fcu_master.mav.set_position_target_local_ned_send(
        0,  # 时间戳，0 表示立即执行
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
        type_mask,
        0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
        0, 0, 0,  # 目标速度
        0, 0, 0,  # 目标加速度（忽略）
        rad, 0  # 目标偏航角度和偏航速率（忽略）
    )
    
def uav_turn_yaw_speed_rad(speed_rad):
    # 北零 左逆负，右顺正 -pi -- pi
    type_mask = 0b0000101111111111  # 忽略位置信息，只设置速度
    fcu_master.mav.set_position_target_local_ned_send(
        0,  # 时间戳，0 表示立即执行
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
        type_mask,
        0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
        0, 0, 0,  # 目标速度
        0, 0, 0,  # 目标加速度（忽略）
        0, speed_rad  # 目标偏航角度和偏航速率（忽略）
    )
    
def uav_turn_yaw_angle(angle):
    type_mask = 0b0000101111111111  # 忽略位置信息，只设置速度
    fcu_master.mav.set_position_target_local_ned_send(
        0,  # 时间戳，0 表示立即执行
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_FLU,  # 使用FLU坐标系
        type_mask,
        0, 0, 0,  # 目标位置（忽略，因为我们设置了速度）
        0, 0, 0,  # 目标速度
        0, 0, 0,  # 目标加速度（忽略）
        angle/57.3, 0  # 目标偏航角度和偏航速率（忽略）
    )

def uav_hover():
    uav_send_speed_FLU(0,0,0)
    
def uav_land():
    for i in range(100):
        uav_send_speed_FLU(0,0,-0.2)
        print("landing")
        time.sleep(0.1)
    
def uav_take_off():
    for i in range(30):
        uav_send_speed_FLU(0,0,0.25)
        print("traking off")
        time.sleep(0.1)
    uav_hover()

def arm_uav():
    fcu_master.mav.command_long_send(
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

def arm_and_takeoff():
    arm_uav()
    time.sleep(0.1)
    arm_uav()
    time.sleep(1)
    uav_take_off()
    
def dis_arm_uav():
    fcu_master.mav.command_long_send(
        fcu_master.target_system, fcu_master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        0,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0  # Unused parameters
    )
    
def data_gcs2fcu_t():
    while True:
        try:
            # 从地面站接收消息
            msg = gcs_master.recv_match(blocking=True)
            if not msg:
                continue

            # 检查消息类型
            if msg.get_type() in ['VISION_POSITION_ESTIMATE', 'SET_POSITION_TARGET_LOCAL_NED', 'HEARTBEAT']:
                # 将消息原样转发到飞控
                continue
            if msg.get_type() == 'HEARTBEAT':
                
                fcu_master.mav.send(msg)
        except KeyboardInterrupt:
            print("程序被用户中断")
            break

def data_fcu2ros_t():
    # rospy.init_node('fcu2ros_node')
    # pub_uav_state = rospy.Publisher()
    
    last_print_time = time.time()
    while True:
        try:
            # 从飞控接收消息
            msg = fcu_master.recv_match(blocking=True)
            print(msg)
            if not msg:
                continue
            # print(msg.get_type())
            if time.time() - last_print_time > 0.5:
                last_print_time = time.time()
                # print(msg.get_type())
                if msg.get_type() == 'HEARTBEAT':
                    print('heart beat')
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    m = msg.to_dict()
                    print(m)
                    # print(f"飞机位置NED："+
                    #     f"x : {m['x'] :.2f} "+
                    #     f"y : {m['y'] :.2f} "+
                    #     f"z : {m['z'] :.2f} "+
                    #     f"vx: {m['vx']:.2f} "+
                    #     f"vy: {m['vy']:.2f} "+
                    #     f"vz: {m['vz']:.2f} "
                    #     )
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                print(msg.to_dict())
            if msg.get_type() == 'ATTITUDE':
                print(f"yaw: {msg.to_dict()['yaw']}")
            # print(msg.get_type())
            # 检查消息类型
            if msg.get_type() in ['VISION_POSITION_ESTIMATE', 'SET_POSITION_TARGET_LOCAL_NED', 'HEARTBEAT']:
                # 将消息原样转发到地面站
                continue
                
        except KeyboardInterrupt:
            print("程序被用户中断")
            break


def test_ctrl_t():
    """测试线程
    """
    arm_uav()
    time.sleep(0.05)
    arm_uav()
    time.sleep(0.05)
    arm_uav()
    time.sleep(0.05)
    uav_take_off()
    for i in range(50):
        uav_hover()
        print("悬停")
        time.sleep(0.1)
    for i in range(50):
        uav_send_speed_FLU(0.3, 0.0, 0.0)
        print("向前飞行")
        time.sleep(0.1)
        
    for i in range(50):
        uav_send_speed_FLU(0.0, -0.3, 0.0)
        print("向右飞行")
        time.sleep(0.1)
    for i in range(50):
        uav_send_speed_FLU(0.0, 0.3, 0.0)
        print("向左飞行")
        time.sleep(0.1)
    uav_land()
    dis_arm_uav()

def test_ctrl_t2():
    """测试线程2
    """
    arm_and_takeoff()
    while True:
        # uav_send_speed_FLU(0.3, 0., 0.)
        # print("向前飞行")
        # time.sleep(0.1)
        # continue
        # arm_uav()
        # for i in range(15):
        #     uav_hover()
        #     print("悬停")
        #     time.sleep(0.1)
        for i in range(30):
            uav_send_speed_FLU(0.3, 0, 0)
            print("向前飞行")
            time.sleep(0.1)
            
        for i in range(30):
            uav_send_speed_FLU(0.0, -0.3, 0.0)
            print("向右飞行")
            time.sleep(0.1)
        for i in range(30):
            uav_send_speed_FLU(0.0, 0.3, 0.0)
            print("向左飞行")
            time.sleep(0.1)
            
        # uav_land()
        # time.sleep(5)
        # dis_arm_uav()

def test_turn():
    angle = 50
    print(f"turn to {angle}")
    uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    print(f"turn to {angle}")
    uav_turn_yaw_angle(angle)
    time.sleep(10)
    angle += 50
    uav_land()

def test_turn_rad():
    rad = 0
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = 3.14/4
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = 3.14/2
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = 3.14*3/4
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = 3.14/2
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = 0
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = -3.14/4
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    rad = -3.14/2
    print(f"turn to {rad}")
    uav_turn_yaw_rad(rad)
    time.sleep(10)
    
    uav_land()
    
if __name__ == "__main__":
    # test_ctrl_t()
    # test_turn()
    # data_fcu2ros_t()
    arm_uav()
    time.sleep(1)
    arm_uav()
    time.sleep(1)
    uav_take_off()
    time.sleep(1)
    uav_land()
    # test_ctrl_t()
    
    # data_fcu2ros_t()
    # print(mavutil.mavlink.MAV_FRAME_LOCAL_FLU)
    # data_fcu2gcs_t()
    # test_master = mavutil.mavlink_connection("udpout:192.168.1.103:9000")
    # while True:
    #     test_master.mav.heartbeat_send(
    #         type=mavutil.mavlink.MAV_TYPE_GCS,  # 类型为地面控制站
    #         autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # 无自动驾驶
    #         base_mode=0,
    #         custom_mode=0,
    #         system_status=mavutil.mavlink.MAV_STATE_UNINIT
    #     )
    #     print(1)
    #     time.sleep(0.5)
    