import rospy
from sailboat_message.msg import WTST_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from sensor_fusion_msg.msg import GpsKF
import message_filters

target_x = [0, 10]
target_y = [0, 0]

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
path_obs_boat = Path()
path_obs_ground = Path()
path_obs_filter_ground = Path()



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
    po_obs.header = obs_pos.header
    po_obs.header.frame_id = 'map'
    po_obs.pose.position.x = obs_pos.point.y  ## change x and y to plot
    po_obs.pose.position.y = obs_pos.point.x
    # print obs_pos.point.y, obs_pos.point.x

    path_obs_boat.poses.append(po_obs)
    path_obs_boat.header.frame_id = 'map'
    path_pub_obs_boat.publish(path_obs_boat)


def obs_ground_cb(obs_pos):
    print 'in obs ground callback'
    po_obs = PoseStamped()
    po_obs.pose.position.x = obs_pos.point.y  ## change x and y to plot
    po_obs.pose.position.y = obs_pos.point.x
    # print obs_pos.point.y, obs_pos.point.x

    if 1:
        row = '{x} {y}\n'.format(x=obs_pos.point.x, y=obs_pos.point.y)
        f1.write(row)

    path_obs_ground.poses.append(po_obs)
    path_obs_ground.header = obs_pos.header
    path_obs_ground.header.frame_id = 'map'
    path_pub_obs_ground.publish(path_obs_ground)


def obs_ground_filter_cb(GPSmsg):
    print 'in obs ground filter callback'
    po_obs = PoseStamped()
    # po_obs.header = GPSmsg.header
    # po_obs.header.frame_id = 'map'
    po_obs.pose.position.x = GPSmsg.posy  ## change x and y to plot
    po_obs.pose.position.y = GPSmsg.posx

    if 1:
        row = '{x} {y}\n'.format(x=GPSmsg.posx, y=GPSmsg.posy)
        f2.write(row)
        row = '{velx} {vely}\n'.format(velx=GPSmsg.velx, vely=GPSmsg.vely)
        f3.write(row)

    path_obs_filter_ground.poses.append(po_obs)
    path_obs_filter_ground.header = GPSmsg.header
    path_obs_filter_ground.header.frame_id = 'map'
    path_pub_obs_filter_ground.publish(path_obs_filter_ground)


def main():
    global path_pub_ego, path_pub_obs_boat, path_pub_obs_ground, points_pub, path_pub_obs_filter_ground
    rospy.init_node('get_path', anonymous=True)

    points_pub = rospy.Publisher('/target_points', Marker, queue_size=2)
    path_pub_ego = rospy.Publisher('/path_ego', Path, queue_size=2)
    path_pub_obs_boat = rospy.Publisher('/path_obs_boat', Path, queue_size=2)
    path_pub_obs_ground = rospy.Publisher('/path_obs_ground', Path, queue_size=2)
    path_pub_obs_filter_ground = rospy.Publisher('/path_obs_filter_ground', Path, queue_size=2)

    # wtst_sub = message_filters.Subscriber('/wtst', WTST_msg)
    # obs_pos_sub = message_filters.Subscriber('/obs_position', Point)
    #
    # ts = message_filters.ApproximateTimeSynchronizer([wtst_sub, obs_pos_sub], queue_size=10,
    #                                                  slop=0.1)  ## slop defines the delay (in seconds) with which messages can be synchronized
    # ts.registerCallback(callback)

    rospy.Subscriber('/wtst', WTST_msg, ego_cb)
    rospy.Subscriber('/obs_boat_position', PointStamped, obs_boat_cb)
    rospy.Subscriber('/obs_ground_position', PointStamped, obs_ground_cb)
    # rospy.Subscriber('/obs_ground_filter_position', PointStamped, obs_ground_filter_cb)
    rospy.Subscriber('/obs_ground_filter_position', GpsKF, obs_ground_filter_cb)


if __name__ == '__main__':
    f1 = open('obs_pos_ground.csv', 'wb')
    f2 = open('obs_pos_ground_filter.csv', 'wb')
    f3 = open('obs_vel_ground_filter.csv', 'wb')
    try:
        main()
        rospy.spin()
    except:
        pass
