import rospy
from sailboat_message.msg import WTST_msg

day = '03'


def callback(wtst_msg):
    timestamp = wtst_msg.UTCtime
    lon = wtst_msg.Longitude
    lat = wtst_msg.Latitude
    row = '{timestamp}{day}, {lon}, {lat}\n'.format(timestamp=timestamp, day=day, lon=lon, lat=lat)
    f.write(row)


def main():
    rospy.init_node('save_tracklet', anonymous=True)

    rospy.Subscriber('/wtst', WTST_msg, callback)


if __name__ == '__main__':
    saveto = 'sjtu_tracklet_09{}.csv'.format(day)
    f = open(saveto, 'wb')
    try:
        main()
        rospy.spin()
    except:
        pass
    finally:
        f.close()
