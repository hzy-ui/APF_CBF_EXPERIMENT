#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from pymavlink import mavutil
import serial


def mavlink_callback(msg):
    # Check if the received MAVLink message is VICON_POSITION_ESTIMATE (message type 62)
    if msg.get_type() == 'VICON_POSITION_ESTIMATE':
        global UAV_pose_rpy
        # Extract VICON_POSITION_ESTIMATE fields
        UAV_pose_rpy.pose.position.x = msg.x
        UAV_pose_rpy.pose.position.y = msg.y
        UAV_pose_rpy.pose.position.z = msg.z
        UAV_pose_rpy.pose.orientation.x = msg.roll
        UAV_pose_rpy.pose.orientation.y = msg.pitch
        UAV_pose_rpy.pose.orientation.z = msg.yaw

        # Process the extracted data
        rospy.loginfo("Received VICON_POSITION_ESTIMATE message:")
        rospy.loginfo("X: %f, Y: %f, Z: %f", x, y, z)
        rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw)
        
        vicon_pub.publish()
    if msg.get_type() == 'LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET':
        
        # Extract VICON_POSITION_ESTIMATE fields
        UAV_pose_rpy.pose.position.x = msg.x
        UAV_pose_rpy.pose.position.y = msg.y
        UAV_pose_rpy.pose.position.z = msg.z
        UAV_pose_rpy.pose.orientation.x = msg.roll
        UAV_pose_rpy.pose.orientation.y = msg.pitch
        UAV_pose_rpy.pose.orientation.z = msg.yaw

        # Process the extracted data
        # rospy.loginfo("Received VICON_POSITION_ESTIMATE message:")
        # rospy.loginfo("X: %f, Y: %f, Z: %f", UAV_pose_rpy.pose.position.x, UAV_pose_rpy.pose.position.y, UAV_pose_rpy.pose.position.z)
        # rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", msg.roll, msg.pitch, msg.yaw)
        
        vicon_pub.publish(UAV_pose_rpy)
        

def main():
    global vicon_pub,UAV_pose_rpy
    UAV_pose_rpy = PoseStamped()
    

    # 初始化ROS节点
    rospy.init_node('gcs_connector')

    # 读取参数，如果没有提供参数则使用默认值
    gcs_conn_protocol = rospy.get_param("~gcs_conn_protocol", "uart")
    gcs_conn_UDP_IP = rospy.get_param("~gcs_conn_UDP_IP", "192.168.1.120")
    gcs_conn_UDP_port = rospy.get_param("~gcs_conn_UDP_port", "5555")
    gcs_conn_baundrate = rospy.get_param("~gcs_conn_baundrate", 115200)
    gcs_conn_serial_port = rospy.get_param("~gcs_conn_serial_port", "/dev/ttyUSB1")

    # 打印读取到的参数
    rospy.loginfo("gcs_conn_protocol: %s", gcs_conn_protocol)
    rospy.loginfo("gcs_conn_UDP_IP: %s", gcs_conn_UDP_IP)
    rospy.loginfo("gcs_conn_UDP_port: %s", gcs_conn_UDP_port)
    rospy.loginfo("gcs_conn_baundrate: %d", gcs_conn_baundrate)
    rospy.loginfo("gcs_conn_serial_port: %s", gcs_conn_serial_port)

    if gcs_conn_protocol.lower() == "uart":
        
        # 设置串口参数
        mav_gcs = mavutil.mavlink_connection(gcs_conn_serial_port, baud=gcs_conn_baundrate)
    elif gcs_conn_protocol.lower() == "udp":
        mav_gcs = mavutil.mavlink_connection('udp:{ip}:{port}'.format(ip=gcs_conn_UDP_IP, port=gcs_conn_UDP_port))
    # 创建ROS Publisher，发布Vicon消息
    vicon_pub = rospy.Publisher('/vicon/pose', PoseStamped, queue_size=10)

    # 进入ROS循环
    while not rospy.is_shutdown():
        try:
            # 从串口接收MAVLink消息
            msg = mav_gcs.recv_msg()
            if msg is not None:
                # 处理接收到的MAVLink消息
                mavlink_callback(msg)
        except (serial.serialutil.SerialException, KeyboardInterrupt):
            rospy.loginfo("在与 GCS 的通讯中遇到 serial 错误，将要退出")
            break
        except:
            rospy.loginfo("在与 GCS 的通讯中遇到严重错误，将要退出")

if __name__ == "__main__":
    main()
