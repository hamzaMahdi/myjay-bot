#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
import math

pts = 0 #to be used to check how many timer prior the ball was seen (as a filter)
got_ball = False


def check_ball(msg):
    global last_seen
    global pts
    global got_ball
    if msg.linear.x !=0:
        pts+=1
        last_seen = rospy.Time.now()
        arrival_threshold = 0.25
        if msg.linear.x<arrival_threshold:
            got_ball = True

# define state WaitForBall
class WaitForBall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_ball','timed_out'])
        self.time_out = 100 # in seconds
        

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitForBall')
        self.initial_time = rospy.Time.now()
        global pts 
        pts = 0
        rate = rospy.Rate(20)
        while (rospy.Time.now()-self.initial_time).to_sec() < self.time_out:
            if pts>10:
                return 'found_ball'
            rate.sleep()
        return 'timed_out'


# define state NavigateToBall
class NavigateToBall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_to_ball','lost_ball'])
        self.time_out = 2
        


    def execute(self, userdata):
        rospy.loginfo('Executing state NavigateToBall')
        global last_seen
        global got_ball
        got_ball = False
        last_seen = rospy.Time.now()
        rate = rospy.Rate(20)
        while (rospy.Time.now()-last_seen).to_sec()<self.time_out:
            if got_ball:
                return 'arrived_to_ball'
            rate.sleep()
        return 'lost_ball'



# main
def main():
    rospy.init_node('find_ball_state_machine')

    # create subscriber to check if ball is found 
    cam_sub = rospy.Subscriber("/camera_cmd_vel", Twist, check_ball)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WaitForBall', WaitForBall(), 
                               transitions={'found_ball':'NavigateToBall', 
                                            'timed_out':'failed'})
        smach.StateMachine.add('NavigateToBall', NavigateToBall(), 
                               transitions={'arrived_to_ball':'WaitForBall',
                                            'lost_ball':'WaitForBall'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('ball_state_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()