#!/usr/bin/env python
import rospy
from sailboat_message.msg import Ahrs_msg
from sailboat_message.msg import GPS_msg
from sailboat_message.msg import WTST_msg

def callback(data):
    #print ('start')
    rospy.loginfo("I heard %f", data.roll)
    #rospy.loginfo("I heard %f", data.WindAngle)

class SensorListener:
    def __init__(self,nodeName,topicName):
        self.NodeName = nodeName
        self.TopicName = topicName

    
    def listener(self):
        rospy.init_node(self.NodeName, anonymous=True)

        #rospy.Subscriber("Ahrs", Ahrs_msg, callback)
        rospy.Subscriber(self.TopicName, Ahrs_msg, callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    ahrs = SensorListener('Ahrslistener','Ahrs')
    ahrs.listener()