from config import *
import rospy
from std_msgs.msg import Int64
rospy.init_node("uav_ctrl_node")
mission_mode_pub = rospy.Publisher("/mission_order", Int64)
while True:
    try:
        # Example message
        print_mode_tip()
        try:
            mode = int(input("mode:"))
        except:
            break
        mission_mode_pub.publish(mode)
    except KeyboardInterrupt:
        print("用户退出")
        break