import rospy
from sailboat_message.msg import WTST_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

## fleet race
# target_x = [0, 58.7091157621, 64.8362530808, 250.098605093, 213.669988671]
# target_y = [0, 115.264402922, 91.6514720561, 112.199262859, 207.218604858], e5: 366.770497586
## station_keeping
target_x = [0, -151.061785525]
target_y = [0, 366.770497586]

path = Path()
marker = Marker()
marker.header.frame_id = 'WTST'
marker.type = marker.POINTS
marker.action = marker.ADD
marker.pose.orientation.w = 1
marker.scale.x = 5.0
marker.scale.y = 5.0
marker.scale.z = 5.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0
marker.color.a = 0.5


for xx, yy in zip(target_x, target_y):
    p = Point()
    p.x = xx
    p.y = yy
    marker.points.append(p)

## Marker cylinder
marker_cylinder = Marker()
marker_cylinder.header.frame_id = 'WTST'
marker_cylinder.type = marker_cylinder.CYLINDER
marker_cylinder.action = marker_cylinder.ADD
marker_cylinder.pose.orientation.w = 1
marker_cylinder.scale.x = 40.0
marker_cylinder.scale.y = 40.0
marker_cylinder.scale.z = 0.1
marker_cylinder.color.r = 0
marker_cylinder.color.g = 0.6
marker_cylinder.color.b = 0
marker_cylinder.color.a = 0.1
marker_cylinder.pose.position.x = -151.061785525
marker_cylinder.pose.position.y = 366.770497586


count = 0
def callback(wtst_msg):
    global count
    po = PoseStamped()
    po.header = wtst_msg.header
    po.pose.position.x = wtst_msg.PosX
    po.pose.position.y = wtst_msg.PosY

    path.poses.append(po)
    if count == 100: #no need to publish path every callback, or the rviz would be very slow
        path.header = wtst_msg.header
        path_pub.publish(path)
        marker.header = wtst_msg.header
        points_pub.publish(marker)
        cylinder_pub.publish(marker_cylinder)
        count = 0
    count += 1


def main():
    global path_pub, points_pub
    rospy.init_node('get_path', anonymous=True)
    path_pub = rospy.Publisher('/path', Path, queue_size=2)
    points_pub = rospy.Publisher('/target_points', Marker, queue_size=2)

    ## for station_keeping
    global cylinder_pub
    cylinder_pub = rospy.Publisher('/target_cylinder', Marker, queue_size=2)

    rospy.Subscriber('/wtst', WTST_msg, callback)




if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except:
        pass
