import rospy
from sailboat_message.msg import WTST_msg

# race = 'fleet_race'
# day = '04'
# race = 'station_keeping'
# day = '05'
race = 'scanning'
day = '06'



def callback(wtst_msg):
    timestamp = wtst_msg.UTCtime
    lon = wtst_msg.Longitude
    lat = wtst_msg.Latitude
    row = '{timestamp}{day}, {lat}, {lon}\n'.format(timestamp=int(timestamp*10), day=day, lat=int(lat*10000000), lon=int(lon*10000000))
    # row = '{timestamp}{day}, {lat}, {lon}\n'.format(timestamp=int(timestamp*10), day=day, lat=lat, lon=lon)
    f.write(row)


def main():
    rospy.init_node('save_tracklet', anonymous=True)

    rospy.Subscriber('/wtst', WTST_msg, callback)


if __name__ == '__main__':
    saveto = 'sjtu_{}_09{}.csv'.format(race, day)
    f = open(saveto, 'wb')
    try:
        main()
        rospy.spin()
    except:
        pass
    finally:
        f.close()
