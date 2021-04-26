#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


def callback_drive(msg):

    #ex: 
    # array = [0,10,0,10,10]
    # my_array_for_publishing = UInt8MultiArray(data=array)

    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z
    #color states based on direction
    #    3: forward, 4: backward, 0: set all 
    #    5: right, 6: left
    #    7: rainbow
    #    second return is for color
    #    this seems inefficient fix later
    if x!=0 and abs(x)>abs(y):
        if x>0:
            array = [4,0,100,0,100]
            led_pub.publish(UInt8MultiArray(data=array))
        else:
            array = [3,100,0,0,100]
            led_pub.publish(UInt8MultiArray(data=array))
    elif y!=0:
        if y>0:
            array = [5,100,0,100,100]
            led_pub.publish(UInt8MultiArray(data=array))
        else:
            array = [6,0,0,100,100]
            led_pub.publish(UInt8MultiArray(data=array))
    else:
        array = [0,0,0,0,100]
        led_pub.publish(UInt8MultiArray(data=array))



# init the ros node and publishers
rospy.init_node('light_feedback')
led_pub = rospy.Publisher("/led", UInt8MultiArray, queue_size =1)
drive_train = rospy.Subscriber("/cmd_vel", Twist, callback_drive)




rospy.spin()

# while not rospy.is_shutdown():
#     flywheel_pub.publish(Int16(data=1500))
#     led_pub.publish(my_array_for_publishing)
#     time.sleep(0.1)
