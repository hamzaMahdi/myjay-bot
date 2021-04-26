#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int16

# init the ros node and publishers
rospy.init_node('test')


flywheel_pub = rospy.Publisher("/flywheel", Int16, queue_size =1)

led_pub = rospy.Publisher("/led", UInt8MultiArray, queue_size =1)

#array = [0,0,0,0,10]
#my_array_for_publishing = UInt8MultiArray(data=array)

while not rospy.is_shutdown():
    #flywheel_pub.publish(Int16(data=1500))
    #array = input('input the meesage\n')
    array = [7,0,0,0,10]
    led_pub.publish(UInt8MultiArray(data=array))
    time.sleep(0.3)
