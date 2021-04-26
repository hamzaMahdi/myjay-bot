#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import  Twist





class LedEmotions:
    def __init__(self):
        # init the ros node and publishers
        rospy.init_node('led_emotions')
        
        self.led_pub = rospy.Publisher("/led", Float32MultiArray, queue_size =1)
        self.vel_pub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size =1)


        self.state = rospy.get_param('/led_driver/emotion', 0)
        # emotion indices
        self.tired_i = 0 
        self.surprised_i = 1 
        self.sad_i = 2 
        self.happy_i = 3
        self.fear_i = 4 
        self.excited_i = 5 
        self.calm_i = 6 
        self.annoyed_i = 7 

        # setup emotions in terms of LED values
        tired =[3.04, -2.06]
        surprised =[1.48, 2.31]
        sad =[-2.38, -1.88]
        happy = [3.45, 0.24]
        fear = [-2.41, -0.68]
        excited = [2.81, 2.16]
        calm = [3.04, -2.79]
        annoyed = [-2.27, 0.72]

        self.led_state = [tired, surprised, sad, happy, fear, excited, calm, annoyed] 
        # setup emotions in terms of velocit
        self.vels = [-2.06, 2.31, -1.88, 0.24, -0.68, 2.16, -2.79, 0.72]
        self.vel_dir = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0]
        self.min_unscaled = min(self.vels)
        self.max_unscaled = max(self.vels)
        map(lambda x:x+self.min_unscaled, self.vels)
        self.min_scaled = min(self.vels)
        self.max_scaled = max(self.vels)

        time.sleep(5)
        print('ready!')

    def drive(self,x,z):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.vel_pub.publish(msg)

    def neutral(self):
        start_time = time.time()
        while(time.time()-start_time<3):
            self.drive(0.5,0)
    
    def emotion(self):
        scaled_vel = (self.vels[self.state]-self.min_scaled)/(self.max_scaled-self.min_scaled)

        start_time = time.time()
        while(time.time()-start_time<(10.0-scaled_vel*5)):
            self.led_pub.publish(Float32MultiArray(data=self.led_state[self.state]))
            self.drive(scaled_vel*self.vel_dir[self.state],0)



experiment = LedEmotions()

print(experiment.state)
experiment.neutral()
experiment.emotion()

print("DONE!")