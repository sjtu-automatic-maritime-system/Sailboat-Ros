import rospy
from sailboat_message.msg import WTST_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

target_x = []
target_y = []
with open('/home/jianyun/catkin_ws/src/Sailboat-Ros/PYTHON/scanning_coord.txt', 'rb') as f:
    for line in f.readlines():
        line = line.split(',')
        target_x.append(float(line[0]))
        target_y.append(float(line[1]))

print target_x
print target_y

path = Path()


marker = Marker()
marker.header.frame_id = 'WTST'
marker.type = marker.LINE_LIST
marker.action = marker.ADD
marker.pose.orientation.w = 1
marker.scale.x = 5.0
marker.scale.y = 5.0
marker.scale.z = 5.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0
marker.color.a = 0.5


pts = []
# for xx, yy in zip(target_x, target_y):
for xx, yy in zip(target_y, target_x): #change x and y to plot
    p = Point()
    p.x = xx
    p.y = yy
    pts.append(p)

marker.points.append(pts[0])
marker.points.append(pts[1])
marker.points.append(pts[2])
marker.points.append(pts[3])
marker.points.append(pts[4])
marker.points.append(pts[5])
marker.points.append(pts[5])
marker.points.append(pts[6])
marker.points.append(pts[6])
marker.points.append(pts[7])
marker.points.append(pts[7])
marker.points.append(pts[8])
marker.points.append(pts[8])
marker.points.append(pts[9])






count = 0
def callback(wtst_msg):
    global count
    po = PoseStamped()
    po.header = wtst_msg.header
    # po.pose.position.x = wtst_msg.PosX
    # po.pose.position.y = wtst_msg.PosY
    po.pose.position.x = wtst_msg.PosY ## change x and y to plot
    po.pose.position.y = wtst_msg.PosX

    path.poses.append(po)
    if count == 100: #no need to publish path every callback, or the rviz would be very slow
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
