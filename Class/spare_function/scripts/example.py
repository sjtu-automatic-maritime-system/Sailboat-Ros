#!/usr/bin/env python
import rospy

from sailboat_message.msg import Sensor_msg
from sailboat_message.msg import Mach_msg

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para

from dynamic_reconfigure.server import Server

sensor_submsg = [0,0,0]
para_cfg = [0,0,0,0,0,0,0,0]

def getOutMachPut(msg): #sailboat_message::Mach_msg
    mach_pub = Mach_msg()
    mach_pub.timestamp = rospy.Time.now()
    mach_pub.motor = 0
    mach_pub.rudder = msg[0]
    mach_pub.sail   = msg[1]
    mach_pub.PCCtrl = msg[2]
    return mach_pub


def getOutput(msg): #spare_function::spare_function_out
    out_pub = spare_function_out()
    out_pub.rudder = msg[0]
    out_pub.sail = msg[1]
    return out_pub


def getOutParaPut(msg):#spare_function::spare_function_para
    para_pubmsg = spare_function_para()
    para_pubmsg.oyaw   = msg[0]
    para_pubmsg.rudderP= msg[1]
    para_pub.rudderI= msg[2]
    para_pub.rudderD= msg[3]
    para_pub.sailP  = msg[4]
    para_pub.sailI  = msg[5]
    para_pub.sailD  = msg[6]
    return out_pub

def sensorCallback(msg): #sailboat_message::Sensor_msg
    global sensor_submsg 
    sensor_submsg[0] = msg.Roll
    sensor_submsg[1] = msg.Yaw
    sensor_submsg[2] = msg.AWA


def getConfigCallback(config, level): #spare_function::spare_function_Config
    global sensor_submsg
    if (config.PC_Ctrl == true):
        para_cfg[0] = 1
    else:
        para_cfg[0] = 0
    para_cfg[1] = config.oyaw
    para_cfg[2] = config.rudderP
    para_cfg[3] = config.rudderI
    para_cfg[4] = config.rudderD
    para_cfg[5] = config.sailP
    para_cfg[6] = config.sailI
    para_cfg[7] = config.sailD
    return config

if __name__ == "__main__":
    rospy.init_node("example", anonymous = True)

    mach_pub = rospy.Publisher('Mach_msg', mach, queue_size=5)
    spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
    spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
    
    rospy.Subscriber("sensor", Sensor_msg, sensorCallback)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10) 

    try:
        while not rospy.is_shutdown():
            mach_np = [0,0,0]
            out_np = [0,0]
            para_np = [0,0,0,0,0,0,0]

            # todo 
            #
            # input : sensor_submsg 
            # cfg: para_cfg
            # output : mach_np out_np para_np

            mach_pubmsg = getOutMachPut(mach_np)
            out_pubmsg = getOutput(out_np)
            para_pubmsg = getOutParaPut(para_np)

            mach_pub.publish(mach_pubmsg)
            spare_function_pub.publish(out_pubmsg)
            spare_function_para_pub.publish(para_pubmsg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        #close()

    rospy.spin()


