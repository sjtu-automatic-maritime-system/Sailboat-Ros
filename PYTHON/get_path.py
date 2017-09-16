import rospy
from sailboat_message.msg import WTST_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import message_filters


target_x = [0, 10]
target_y = [0, 10]

marker = Marker()
marker.header.frame_id = 'map'
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
path_ego = Path()
path_obs = Path()

# obs_pos = PointStamped()
# obs_pos.point.x

# def callback(wtst_msg, obs_pos):
#     global count
#     po_ego = PoseStamped()
#     po_ego.header = wtst_msg.header
#     # po.pose.position.x = wtst_msg.PosX
#     # po.pose.position.y = wtst_msg.PosY
#     po_ego.pose.position.x = wtst_msg.PosY  ## change x and y to plot
#     po_ego.pose.position.y = wtst_msg.PosX
#     path_ego.poses.append(po_ego)
#
#     po_obs = PoseStamped()
#     po_obs.header = wtst_msg.header
#     # po_obs.position.x = obs_pos.x
#     # po_obs.position.y = obs_pos.y
#     po_obs.pose.position.x = obs_pos.point.y  ## change x and y to plot
#     po_obs.pose.position.y = obs_pos.point.y
#     path_obs.poses.append(po_obs)
#
#
#     if count == 10:  # no need to publish path every callback, or the rviz would be very slow
#         path_ego.header = wtst_msg.header
#         path_pub_ego.publish(path_ego)
#         path_obs.header = wtst_msg.header
#         # path_obs.header.frame_id = 'WTST'
#         path_pub_obs.publish(path_obs)
#         marker.header = wtst_msg.header
#         points_pub.publish(marker)
#         count = 0
#     count += 1


def ego_cb(wtst_msg):
    print 'in ego callback'
    global count
    po_ego = PoseStamped()
    po_ego.header = wtst_msg.header
    po_ego.header.frame_id = 'map'
    # po.pose.position.x = wtst_msg.PosX
    # po.pose.position.y = wtst_msg.PosY
    po_ego.pose.position.x = wtst_msg.PosY  ## change x and y to plot
    po_ego.pose.position.y = wtst_msg.PosX

    if count == 10:  # no need to publish path every callback, or the rviz would be very slow
        path_ego.poses.append(po_ego)
        path_ego.header = wtst_msg.header
        path_pub_ego.publish(path_ego)
        marker.header = wtst_msg.header
        points_pub.publish(marker)
        count = 0
    count += 1

#
def obs_boat_cb(obs_pos):
    print 'in obs boat callback'
    po_obs = PoseStamped()
    po_obs.header = po_obs.header
    po_obs.header.frame_id = 'map'
    po_obs.pose.position.x = obs_pos.point.y  ## change x and y to plot
    po_obs.pose.position.y = obs_pos.point.x
    print obs_pos.point.y, obs_pos.point.x

    path_obs.poses.append(po_obs)
    path_obs.header.frame_id = 'map'
    path_obs.header = obs_pos.header
    path_pub_obs_boat.publish(path_obs)

def obs_ground_cb(obs_pos):
    print 'in obs ground callback'
    po_obs = PoseStamped()
    po_obs.header = po_obs.header
    po_obs.header.frame_id = 'map'
    # po_obs.position.x = obs_pos.x
    # po_obs.position.y = obs_pos.y
    po_obs.pose.position.x = obs_pos.point.y  ## change x and y to plot
    po_obs.pose.position.y = obs_pos.point.x
    print obs_pos.point.y, obs_pos.point.x

    path_obs.poses.append(po_obs)
    path_obs.header.frame_id = 'map'
    path_obs.header = obs_pos.header
    path_pub_obs_ground.publish(path_obs)



def main():
    global path_pub_ego, path_pub_obs_boat, path_pub_obs_ground, points_pub
    rospy.init_node('get_path', anonymous=True)

    points_pub = rospy.Publisher('/target_points', Marker, queue_size=2)
    path_pub_ego = rospy.Publisher('/path_ego', Path, queue_size=2)
    path_pub_obs_boat = rospy.Publisher('/path_obs_boat', Path, queue_size=2)
    path_pub_obs_ground = rospy.Publisher('/path_obs_ground', Path, queue_size=2)

    # wtst_sub = message_filters.Subscriber('/wtst', WTST_msg)
    # obs_pos_sub = message_filters.Subscriber('/obs_position', Point)
    #
    # ts = message_filters.ApproximateTimeSynchronizer([wtst_sub, obs_pos_sub], queue_size=10,
    #                                                  slop=0.1)  ## slop defines the delay (in seconds) with which messages can be synchronized
    # ts.registerCallback(callback)

    rospy.Subscriber('/wtst', WTST_msg, ego_cb)
    # rospy.Subscriber('/obs_boat_position', PointStamped, obs_boat_cb)
    rospy.Subscriber('/obs_ground_position', PointStamped, obs_ground_cb)


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except:
        pass
