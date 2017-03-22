#!/usr/bin/env python
import rospy
from sailboat_message.msg import Ahrs_msg
from sailboat_message.msg import GPS_msg
from sailboat_message.msg import WTST_msg

def callback(data):
    #print ('start')
    #rospy.loginfo("I heard %f", data.roll)
    rospy.loginfo("I heard %f", data.WindAngle)

    #print('get')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("Ahrs", Ahrs_msg, callback)
    rospy.Subscriber("WTST", WTST_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()