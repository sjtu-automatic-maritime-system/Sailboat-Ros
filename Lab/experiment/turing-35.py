#!/usr/bin/env python
import signal, sys
import rospy
from sailboat_message.msg import Ahrs_msg
from sailboat_message.msg import Mach_msg
import time
# from std_msgs.msg import Float64
class listener:
    def __init__(self):
        self.isinit=True
    def callback(self,data):
        self.yaw=data.yaw

    
    def listen(self):
        rospy.Subscriber('ahrs', Ahrs_msg, self.callback)

class Experiment:
    def __init__(self):
        self.state=-1
    def zig_zag(self,yaw,yaw_start,rudder):
        if yaw-yaw_start>5.0:
            yaw=yaw-3.14159*2
        elif yaw-yaw_start<-5.0:
            yaw=yaw+3.14159*2
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

class dataWrapper:
    """docstring for dataWrapper"""
    def __init__(self):
        self.motor='motor'
        self.rudder='rudder'


    def pubData(self,msg,rudder,motor):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'Mach'


        msg.rudder=rudder
        msg.motor=motor
        msg.PCCtrl=1
        msg.sail=1
        return msg



def talker(rudder,motor):
    signal.signal(signal.SIGINT,quit)
    signal.signal(signal.SIGTERM,quit)


    pub = rospy.Publisher('mach', Mach_msg, queue_size=10)  
    rospy.init_node('rudder_talker', anonymous=True)  
    rate = rospy.Rate(10) # 10hz 
    msg = Mach_msg()
    datawrapper = dataWrapper()

    ahrs_listener=listener()
    experiment=Experiment()
    start_time=time.time()
    # try:
    ahrs_listener.listen()
    time.sleep(3)
    while not rospy.is_shutdown():  
        if time.time()-start_time<10:
            mach_msg=datawrapper.pubData(msg,0,motor)
            try:
                yaw_start=ahrs_listener.yaw
                print(yaw_start)
            except:
                pass
        else:
            set_rudder=experiment.turning(rudder)
            if time.time()-start_time>50:
                motor=0
            mach_msg=datawrapper.pubData(msg,set_rudder,motor)
            print(ahrs_listener.yaw,yaw_start,set_rudder)
        pub.publish(mach_msg)  
        rate.sleep()
    # except rospy.ROSInterruptException:
    #     pass
    # finally:
    #     raise

def quit(signum,frame):
    print('byebye')
    sys.exit()

if __name__ == '__main__':
    talker(-35*0.01745,20)
       


       

