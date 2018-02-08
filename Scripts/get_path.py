import rospy
from sailboat_message.msg import WTST_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

path = Path()

target_x = [0, -151.061785525]
target_y = [0, 366.770497586]

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

count = 0


def callback(wtst_msg):
    global count
    po = PoseStamped()
    po.header = wtst_msg.header
    # po.pose.position.x = wtst_msg.PosX
    # po.pose.position.y = wtst_msg.PosY
    po.pose.position.x = wtst_msg.PosY  ## change x and y to plot
    po.pose.position.y = wtst_msg.PosX

    path.poses.append(po)
    if count == 100:  # no need to publish path every callback, or the rviz would be very slow
        path.header = wtst_msg.header
        path_pub.publish(path)
        marker.header = wtst_msg.header
        points_pub.publish(marker)
        count = 0
    count += 1


def main():
    global path_pub, points_pub
    rospy.init_node('get_path', anonymous=True)
    path_pub = rospy.Publisher('/path', Path, queue_size=2)
    points_pub = rospy.Publisher('/target_points', Marker, queue_size=2)

    rospy.Subscriber('/wtst', WTST_msg, callback)


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except:
        pass
