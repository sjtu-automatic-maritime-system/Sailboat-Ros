#!/usr/bin/env python
import signal, sys
import rospy
from sailboat_message.msg import Ahrs_msg
from std_msgs.msg import Float64
class listener:
    def __init__(self):
        self.yaw_start=0
        self.isinit=True
        pass
    def callback(self,data):
        if self.isinit==True:
            self.yaw_start=data.yaw
            self.yaw=data.yaw
            self.isinit=False
        self.yaw=data.yaw

        # if self.isinit==True:
        #     self.yaw_start=data.data
        #     self.yaw=data.data
        #     self.isinit=False
        # self.yaw=data.data
     

    

    def listen(self):
        rospy.Subscriber('ahrs', Ahrs_msg, self.callback)
        # rospy.Subscriber('chatter', Float64, self.callback)

    
    
    # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()
class Experiment:
    def __init__(self):
        self.state=-1
    def zig_zag(self,yaw,yaw_start,rudder):
        if yaw-yaw_start>rudder:
            self.state=1
        elif yaw-yaw_start<-rudder:
            self.state=-1
        if self.state==1:
            set_rudder=-rudder
        elif self.state==-1:
            set_rudder=rudder
        return set_rudder
    def turning(self,rudder):
        return rudder

        
def talker(mode,ruuder):
    signal.signal(signal.SIGINT,quit)
    signal.signal(signal.SIGTERM,quit)
    # mode=input("select mode turning or zig_zag:")
	# rudder=float(input("select rudder angle:"))


    pub = rospy.Publisher('rudder', Float64, queue_size=10)  
    rospy.init_node('rudder_talker', anonymous=True)  
    rate = rospy.Rate(10) # 10hz 

    # mode='zig_zag'
    # rudder=0.52
    ahrs_listener=listener()
    experiment=Experiment()
    try:
        ahrs_listener.listen()
        while not rospy.is_shutdown():  
            while ahrs_listener.yaw_start!=0:
                print(ahrs_listener.yaw,ahrs_listener.yaw_start)
                if mode=='zig_zag':
                    set_rudder=experiment.zig_zag(ahrs_listener.yaw,ahrs_listener.yaw_start,rudder)
                elif mode=='turning':
                    set_rudder=experiment.turning(rudder)
                print(ahrs_listener.yaw,ahrs_listener.yaw_start,set_rudder)
                pub.publish(set_rudder)  
                rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        raise

def quit(signum,frame):
    print('byebye')
    sys.exit()

if __name__ == '__main__':
    mode='zig_zag'
    rudder=0.52
    talker(mode,rudder)
       

