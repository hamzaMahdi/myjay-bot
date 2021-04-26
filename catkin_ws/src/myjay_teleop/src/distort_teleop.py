#!/usr/bin/env python
import rospy
import time 
from geometry_msgs.msg import Twist
from smach_msgs.msg import SmachContainerStatus
import numpy as np
import math
from collections import deque


class TeleopDistorter:
    def __init__(self):
        cmd_sub = rospy.Subscriber("/undistorted/cmd_vel", Twist, self.distort_vel)
        state_sub = rospy.Subscriber("/ball_state_server/smach/container_status",\
            SmachContainerStatus,self.update_state)
        self.publish_distorted_vel = rospy.Publisher("/cmd_vel", Twist, queue_size =1)
        self.publish_distorted_vel_debug = rospy.Publisher("/distorted/cmd_vel", Twist, queue_size =1)

        self.goal_deviation = rospy.get_param('/goal_deviation', default=0)# how much x-y components leak into eachother (percentage)
        self.noise_level = rospy.get_param('/noise_level', default=0)# noise at start and end of the trajectory 
        self.force = rospy.get_param('/force', default=1)# force user is able to apply (percentage)
        self.time_delay = rospy.get_param('/time_delay', default=0)# response delay in samples
        self.delayed_commands = deque()
        self.last_cmd = rospy.Time.now()# to discard very old messages

        self.state = "WaitForBall"
        self.prev_state = self.state
        self.last_state_change = rospy.Time.now()

    def update_state(self,msg):
        prev_state = self.state
        self.state = msg.active_states[0]
        #add state machine check for rotation noise
        if self.state != self.prev_state:
            self.last_state_change = rospy.Time.now()
        print("current state: " + self.state)
    
    def distort_vel(self,msg):
        if (rospy.Time.now()-self.last_cmd).to_sec()>2:
            self.delayed_commands.clear() # clear messages older than 2 seconds
        self.last_cmd = rospy.Time.now()

        # append new command
        self.delayed_commands.append(msg)

        data = Twist()
        if len(self.delayed_commands) >= self.time_delay:
            data = self.delayed_commands.popleft() # get oldest command

            if abs(data.linear.x)>abs(data.linear.y):
                # the bigger one is already reduced so force is only multiplied once
                data.linear.x*=self.force
                data.linear.y+=data.linear.x*self.goal_deviation
                #data.angular.z*=distort_scale
            else:
                data.linear.y*=self.force
                data.linear.x+=data.linear.y*self.goal_deviation
            
            # weaken the angular vel
            data.angular.z*=self.force
            # distort for 2 seconds
            # if (rospy.Time.now()-self.last_state_change).to_sec()<2:
            data.angular.z += np.random.normal(0,1,1)[0]*self.noise_level # add gaussian noise

        else:
            data.linear.x*=0
            data.linear.y*=0
            data.angular.z*=0
        self.publish_distorted_vel.publish(data)
        self.publish_distorted_vel_debug.publish(data)



if __name__=='__main__':
    rospy.init_node('distort_teleop')
    distorter = TeleopDistorter()
    rospy.spin()