#!/usr/bin/env python

from socket import *
import sys
import select
from time import sleep
from time import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int16
import thread

# networking parameters
host="10.42.0.1"
#host  = "192.168.0.70"
port = 8090
s = socket(AF_INET,SOCK_DGRAM)
s.bind((host,port))
addr = (host,port)
buf=1024


# init the ros node and publishers
rospy.init_node('supervisor_udp_reciever')
collect_data = rospy.get_param('/collect_data', default=False)# for rosbag data collection 


# publish drivetrain twist to cmd_vel
def actuate_robot(vel_pub, x, y, z):
    max_vel = 70 # max is 114cm/s
    max_angular_rate = 3 # rad/s 
    msg = Twist()
    msg.linear.x = (x-max_vel)/100.0
    msg.linear.y = (y-max_vel)/100.0 # convert to m/s
    msg.angular.z = z-max_angular_rate

    vel_pub.publish(msg)

    if collect_data:
        training_vel_pub.publish(msg)
    

    # TODO: create a separate LED subscriber

    
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



vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size =10)
training_vel_pub = rospy.Publisher("/supervisor_cmd_vel", Twist, queue_size =10)

# for now intake and elevator are on/off one direction 
intake_pub = rospy.Publisher("/intake", Int16, queue_size =10)
elevator_pub = rospy.Publisher("/elevator", Int16, queue_size =10)

# might get changed
led_pub = rospy.Publisher("/led", UInt8MultiArray, queue_size =1)
flywheel_pub = rospy.Publisher("/flywheel", Int16, queue_size =1)

# flywheel PWM parameters
flywheel = 1500
acceleration = 10
accelerate = False


array = [0,0,0,0,10]
my_array_for_publishing = UInt8MultiArray(data=array)

led_pub.publish(my_array_for_publishing)


def recieve_msg(vel_pub, intake_pub, elevator_pub):
    while True:
        data,addr = s.recvfrom(buf)
        #print "Received File:", data.strip()
        #print addr
        last_seen = time()
        msg = []
        message_size=6 # n_commands sent,probably wont change
        for i in range(message_size):
            msg.append(int(data.strip()[i*3+1:4+i*3]))# inefficient, fix later
        # big publisher function
        actuate_robot(vel_pub, msg[0], msg[1], msg[2])
        intake_pub.publish(msg[3])
        elevator_pub.publish(msg[4])
        global accelerate 
        accelerate = msg[5]
        


try:
   thread.start_new_thread( recieve_msg, (vel_pub, intake_pub, elevator_pub,) )
except:
   print "Error: unable to start udp thread"

rospy.spin()

# dont do flywheel for supervisor (or add a mux option) 
''' 
while not rospy.is_shutdown():
    
    if accelerate and flywheel >1110:
        flywheel-=acceleration # negative because throw is <1500 uS
    elif flywheel<1500:
        flywheel+=acceleration
    flywheel_pub.publish(Int16(flywheel))
    sleep(0.05)
'''