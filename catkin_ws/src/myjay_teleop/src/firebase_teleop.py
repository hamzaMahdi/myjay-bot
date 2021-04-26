#!/usr/bin/env python

from socket import *
import sys
import select
from time import sleep
from time import time

# firebase interface
import pyrebase
config = {
  #config removed for privacy
}

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int16

class FirebaseTeleop:

    def __init__(self):
        # initiate connection
        self.firebase = pyrebase.initialize_app(config)
        self.db = self.firebase.database()


        # init the ros publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size =10)

        # for now intake and elevator are on/off one direction 
        self.intake_pub = rospy.Publisher("/intake", Int16, queue_size =10)
        self.elevator_pub = rospy.Publisher("/elevator", Int16, queue_size =10)

        # might get changed
        self.led_pub = rospy.Publisher("/led", UInt8MultiArray, queue_size =1)
        self.flywheel_pub = rospy.Publisher("/flywheel", Int16, queue_size =1)

        # flywheel PWM parameters
        self.flywheel = 1500
        self.acceleration = 10
        self.accelerate = 0

        # turn off LED
        array = [0,0,0,0,10]
        my_array_for_publishing = UInt8MultiArray(data=array)
        self.led_pub.publish(my_array_for_publishing)


    # publish drivetrain twist to cmd_vel
    def actuate_robot(self, x, y, z):

        # safe velocity limits, might change later
        max_vel = 60 # max is 114cm/s
        max_angular_rate = 3 # rad/s 
        msg = Twist()
        msg.linear.x = (x-max_vel)/100.0
        msg.linear.y = (y-max_vel)/100.0 # convert to m/s
        msg.angular.z = z-max_angular_rate

        self.vel_pub.publish(msg)
        
        # if flywheel:
        #     flywheel_pub.publish(Int16(data=1400))
        # else:
        #     flywheel_pub.publish(Int16(data=1500))
        # color states based on direction
        # 3: forward, 4: backward, 0: set all 
        # second return is for color
        # this seems inefficient fix later
        # if x!=0:
        #     if x>0:
        #         array = [4,100,0,0,100]
        #         led_pub.publish(UInt8MultiArray(data=array))
        #     else:
        #         array = [3,100,0,0,100]
        #         led_pub.publish(UInt8MultiArray(data=array))
        # elif y!=0:
        #     if y>0:
        #         array = [0,100,0,100,100]
        #         led_pub.publish(UInt8MultiArray(data=array))
        #     else:
        #         array = [0,0,0,100,100]
        #         led_pub.publish(UInt8MultiArray(data=array))
        # else:
        #     array = [5,0,0,0,100]
        #     led_pub.publish(UInt8MultiArray(data=array))





    def stream_handler(self, message):
        print(message["event"]) # put
        print(message["path"]) # /-K7yGTTEp7O549EzTYtI
        
        data = message["data"].get('name')
        print(data) # {'title': 'Pyrebase', "body": "etc..."}
        if data != None:
            last_seen = time()
            msg = []
            message_size=6 # n_commands sent,probably wont change
            for i in range(message_size):
                msg.append(int(data.strip()[i*3+1:4+i*3]))# inefficient, fix later
            
            self.accelerate = msg[5]
            
            # big publisher function
            self.actuate_robot(msg[0], msg[1], msg[2])
            self.intake_pub.publish(msg[3])
            self.elevator_pub.publish(msg[4])
    






if __name__ == '__main__':
    rospy.init_node('firebase_reciever')

    node = FirebaseTeleop()
    my_stream = node.db.child("users").stream(node.stream_handler)
    while not rospy.is_shutdown():
        if(node.accelerate) and node.flywheel >1110:
            node.flywheel-=node.acceleration # negative because throw is <1500 uS
        elif node.flywheel<1500:
            node.flywheel+=node.acceleration
        node.flywheel_pub.publish(Int16(node.flywheel))
        sleep(0.05)
    my_stream.close()

