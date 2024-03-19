import time
import rospy
from visualization_msgs.msg import Marker

# 定义圆柱体 Marker 的尺寸
CYLINDER_HEIGHT = 1.0
CYLINDER_RADIUS = 0.5

def publish_cylinders(positions_and_radiuses):
    """
    发布多个圆柱体 Markers.

    Args:
        positions_and_radiuses: 一个列表，包含每个圆柱体的位置和半径的三维元祖.
    """

    markers_pub = rospy.Publisher("/cylinders", Marker, queue_size=1)

    for i, (position, radius) in enumerate(positions_and_radiuses):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cylinders"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = CYLINDER_HEIGHT
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        markers_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("publish_cylinders")

    # 定义圆柱体的位置和半径
    positions_and_radiuses = [
        ((1.0, 2.0, 0.0), 0.5),
        ((3.0, 4.0, 0.0), 1.0),
        ((5.0, 6.0, 0.0), 1.5),
    ]
    x1 = 0
    x2 = 0
    x3 = 0
    
    y1 = 1
    y2 = 5
    y3 = 9
    # 发布圆柱体 Markers
    while True:
        x1 += 0.1
        x2 += 0.1
        x3 += 0.1
        
        y1 += 0.1
        y2 += 0.1
        y3 += 0.1
        positions_and_radiuses = [
        ((x1, y1, 0.0), 0.5),
        ((x2, y2, 0.0), 1.0),
        ((x3, y3, 0.0), 1.5),
    ]
        publish_cylinders(positions_and_radiuses)
        time.sleep(0.1)

    # 等待关闭
    rospy.spin()
