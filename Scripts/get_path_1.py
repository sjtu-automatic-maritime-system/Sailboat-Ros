import rospy
from sailboat_message.msg import WTST_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

path = Path()


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
    if count == 10:  # no need to publish path every callback, or the rviz would be very slow
        path.header = wtst_msg.header
        path.header.frame_id = 'world'
        path_pub.publish(path)
        count = 0
    count += 1


def main():
    global path_pub, points_pub
    rospy.init_node('get_path', anonymous=True)
    path_pub = rospy.Publisher('/ego_path', Path, queue_size=2)

    rospy.Subscriber('/wtst', WTST_msg, callback)


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except:
        pass
