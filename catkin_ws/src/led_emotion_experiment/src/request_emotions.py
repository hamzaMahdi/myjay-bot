#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray

# init the ros node and publishers
rospy.init_node('led_emotions')

led_pub = rospy.Publisher("/led", Float32MultiArray, queue_size =1)


# message structure: func,r,g,b,wave_delay, loop count
msg = [0]*2
while not rospy.is_shutdown():
    command = input("input command to be send to the mcu in the format: func,r,g,b,wave_delay, loop count\n")
    for i in range(len(msg)):
        msg[i] = command[i]
    led_pub.publish(Float32MultiArray(data=msg))
    time.sleep(0.1)
