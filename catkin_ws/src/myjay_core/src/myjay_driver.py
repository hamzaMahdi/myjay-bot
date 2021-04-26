#!/usr/bin/env python
import rospy
import time 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
import serial
from myjay_core.roboclaw import Roboclaw
import numpy as np
import math
import subprocess
import os
stop  = False



# loosly based off mecanumbot and roboclaw_ros packages
class EncoderOdom:
    def __init__(self):

        rev_per_meter = 1/(math.pi*0.1)# 100 mm diameter wheel
        motor_rev_per_meter = rev_per_meter*2 #2:1 gear ratio
        ticks_per_meter = motor_rev_per_meter*383.6# 383.6 encoder counts per motor rev
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = 0.152*2 # 15.2 cm from wheel to center of robot
        # scales need calibration
        self.scale_x = 1
        self.scale_y = 1
        self.odom_pub = rospy.Publisher('/encoder_odom', Odometry, queue_size=10)
        self.actual_yaw_sub = rospy.Subscriber("/mpu_imu", Imu, self.get_actual_angle)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.actual_angle = 0.0
        self.last_enc_left1 = 0
        self.last_enc_left2 = 0
        self.last_enc_right1 = 0
        self.last_enc_right2 = 0
        self.last_enc_time = rospy.Time.now().secs

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # IMU subscriver callback
    def get_actual_angle(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.actual_angle = yaw

    def update(self, right_enc1, right_enc2, left_enc1, left_enc2):
        delta_left_ticks1 = left_enc1 - self.last_enc_left1
        delta_left_ticks2 = left_enc2 - self.last_enc_left2
        delta_right_ticks1 = right_enc1 - self.last_enc_right1
        delta_right_ticks2 = right_enc2 - self.last_enc_right2
        
        self.last_enc_left1 = left_enc1
        self.last_enc_left2 = left_enc2
        self.last_enc_right1 = right_enc1
        self.last_enc_right2 = right_enc2

        # convert the motor counts into x, y, theta counts
        dx = (delta_left_ticks1+ delta_right_ticks1 + delta_left_ticks1 + delta_right_ticks2) / 4.0
        dy = (0 -delta_left_ticks1+ delta_right_ticks1 + delta_left_ticks2 - delta_right_ticks2) / 4.0
        dx = dx*self.scale_x/self.TICKS_PER_METER
        dy = -dy*self.scale_y/self.TICKS_PER_METER
        
        # little fishy, will investigate later
        # https://github.com/joshvillbrandt/mecanumbot-ros-pkg/blob/master/arduino/libraries/Mecanum/Mecanum.cpp
        #dtheta = ( delta_left_ticks1- delta_right_ticks1 + delta_left_ticks1 - delta_right_ticks2) / 4.0
        #dtheta *=self.TICKS_PER_METER/self.BASE_WIDTH

        # more of a differential system. May not be correct
        dist_right = -(delta_right_ticks1+delta_right_ticks2)/ 2.0 / self.TICKS_PER_METER
        dist_left = (delta_left_ticks1+delta_left_ticks2) / 2.0/ self.TICKS_PER_METER
        dtheta = (dist_right - dist_left) / self.BASE_WIDTH

        dist = (dist_right + dist_left) / 4.0

        current_time = rospy.Time.now().secs
        d_time = (current_time - self.last_enc_time)
        self.last_enc_time = current_time


    
        self.cur_x += dx * math.cos(self.actual_angle)-dy*math.sin(self.actual_angle)
        self.cur_y += dx * math.sin(self.actual_angle)+dy*math.cos(self.actual_angle)
        self.cur_theta = self.normalize_angle(self.cur_theta + dtheta) # probably garbage value

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_y = 0.0
            vel_theta = 0.0
        else:
            vel_x = dx / d_time
            vel_y = dy/d_time
            vel_theta = dtheta / d_time

        return vel_x, vel_y, vel_theta

    def update_publish(self, right_enc1, right_enc2, left_enc1, left_enc2):
        # TODO: deal with encoder noise
        vel_x, vel_y, vel_theta = self.update(right_enc1, right_enc2, left_enc1, left_enc2)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_y, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vy, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.actual_angle)
        current_time = rospy.Time.now()

        # replaced by robot_localization
        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.actual_angle),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)




class MyJay:
    def __init__(self):
        self.counter = 0
        self.init_robot()
        # self.odom_pub = rospy.Publisher("/odom", Int64, queue_size=10)# work in progress
        self.drive_train = rospy.Subscriber("/cmd_vel", Twist, self.callback_drive_robot)
        self.intake = rospy.Subscriber("/intake", Int16, self.callback_intake)
        self.elevator = rospy.Subscriber("/elevator", Int16, self.callback_elevator)

        # start with motors not moving
        self.intake_pwm = 64
        self.elevator_pwm = 64

        # health and heartbeat checks
        self.last_cmd_time = rospy.get_rostime().secs
        self.battery_pub = rospy.Publisher("/battery_voltage", Float32, queue_size =1)
        self.led_pub = rospy.Publisher("/led", UInt8MultiArray, queue_size =1)


        self.encoder_odom = EncoderOdom()


        #self.led_sub = rospy.Subscriber("/led", String, self.callback_set_led)

    # initiate roboclaws automatically. Needs prior knowledge of each address 
    def init_robot(self):
        # the arduino will ALWAYS be ttyUSB0 unless unplugged during operation
        # self.ser = serial.Serial('/dev/ttyUSB0',9600)

        # this depends on the physical connections in your robot
        self.right_address = 0x80
        self.left_address = 0x82
        self.intake_address = 0x81
        addresses = [self.right_address,self.left_address,self.intake_address]
        # it's better to create a udev rule with specific names but they all have the same attributes
        self.roboclaw = []
        self.roboclaw.append(Roboclaw("/dev/ttyACM0", 38400))
        self.roboclaw.append(Roboclaw("/dev/ttyACM1", 38400))
        self.roboclaw.append(Roboclaw("/dev/ttyACM2", 38400))


        print('setting up connections...')
        for rc_ind, rc in enumerate(self.roboclaw):
            rc.Open()
            for add_ind, address in enumerate(addresses):
                if(rc.ReadNVM(address) and address==self.right_address):
                    self.right_ind = rc_ind
                    break
                if(rc.ReadNVM(address) and address==self.intake_address):
                    self.intake_ind = rc_ind
                    break
                if(rc.ReadNVM(address) and address==self.left_address):
                    self.left_ind = rc_ind
                    break
        print('setup complete !')
        print('Battery Voltage Level:')
        self.battery_voltage = self.roboclaw[self.right_ind].ReadMainBatteryVoltage(self.right_address)[1]/10.0
        print(self.battery_voltage)

        # reset encoders
        self.stop_drive()
        self.roboclaw[self.right_ind].ResetEncoders(self.right_address)
        self.roboclaw[self.left_ind].ResetEncoders(self.left_address)
        self.drive_motor_pwm = [64,64,64,64] # stop
    

    # drive motors PWM
    def drive_pwm(self, l1, l2, r1, r2):
        self.roboclaw[self.left_ind].ForwardBackwardM1(self.left_address, l1)
        self.roboclaw[self.left_ind].ForwardBackwardM2(self.left_address, l2)
        self.roboclaw[self.right_ind].ForwardBackwardM2(self.right_address, r1)
        self.roboclaw[self.right_ind].ForwardBackwardM1(self.right_address, r2)

    # intake motor PWM
    def run_intake(self, Mspeed):
        self.roboclaw[self.intake_ind].ForwardBackwardM1(self.intake_address, Mspeed)

    # elevator motor PWM
    def run_elevator(self, Mspeed):
        self.roboclaw[self.intake_ind].ForwardBackwardM2(self.intake_address, Mspeed)

    # Stop all drive motors (usually in timeouts)
    def stop_drive(self):
        self.roboclaw[self.left_ind].ForwardM1(self.left_address, 0)
        self.roboclaw[self.left_ind].ForwardM2(self.left_address, 0)
        self.roboclaw[self.right_ind].ForwardM1(self.right_address, 0)
        self.roboclaw[self.right_ind].ForwardM2(self.right_address, 0)

    # needs PID tuning but the firmware is messed up for now
    def drive_vel(self, l1, l2, r1, r2):
        print("l1: ",l1,", l2: ",l2,", r1: ",r1,", r2: ",r2)
        self.roboclaw[self.left_ind].SpeedM1M2(self.left_address, l1, l2)
        self.roboclaw[self.right_ind].SpeedM1M2(self.right_address, r1, r2)

    # receive drivetrain commands in m/s
    def callback_drive_robot(self, msg):
        self.last_cmd_time = rospy.get_rostime()
        # kinematics formula can be found in the modern robotics textbook 
        # http://hades.mech.northwestern.edu/index.php/Modern_Robotics
        # all units are in meters
        l = 0.168 #distance between center of drivetrain and center of one of the wheels 
        w = 0.152 # distance half the distance between two opposite wheels ex(rear left and rear right)
        wheel_radius = 0.05 
        max_rps = 435.0/2/60
        max_ticks = max_rps*383.6 # quadrature enc/s
        max_omega = 435.0/2*2*math.pi/60 # 435 rpm motor with 2:1 gear ratio and convert to rad/s
        max_linear = max_omega*wheel_radius
        H = np.array([[-l-w,1,-1],[l+w,1,1],[l+w,1,-1],[-l-w,1,1]])
        V = np.array([[msg.angular.z],[msg.linear.x],[msg.linear.y]])
        u = 1/wheel_radius*H.dot(V)
        u_inverted = np.ravel(np.multiply(u.T,np.array([1,1,-1,-1]))) # invert motors (due to orientation)
        
        # scale and prepare for PWM (include feedback later)
        # the scale is VERY wrong but i put it here temporarily
        # u_inverted = np.interp(u_inverted, (-max_omega, max_omega), (0, 128)).astype(int)
        # u1, u2, u3, u4
        # front left, front right, back right, back left
        # print(u_inverted)
        
        self.drive_motor_pwm = np.interp(u_inverted, (-max_omega, max_omega), (0, 128)).astype(int)


        

        #u_inverted = np.interp(u_inverted, (-max_omega, max_omega), (-max_ticks, max_ticks)).astype(int)
        #self.drive_vel(u_inverted[0], u_inverted[3], u_inverted[1], u_inverted[2])

    # updsate intake pwm
    def callback_intake(self, run):
        self.intake_pwm = run.data

    # update elevator pwm
    def callback_elevator(self, run):
        self.elevator_pwm = run.data
    
    # update battery voltage and encoders
    def update_sensor_data(self):
        # any of the roboclaws work
        self.battery_voltage = self.roboclaw[self.right_ind].ReadMainBatteryVoltage(self.right_address)[1]/10.0

        # if the battery is too low show red on LED strip
        if self.battery_voltage<10:
            array = [0,100,0,0,100]
            self.led_pub.publish(UInt8MultiArray(data=array))
        
        self.battery_pub.publish(self.battery_voltage)


        # Read encoder
        # note:
        # Each motor rev is 383.6 ticks, 2:1 gear ratio-> 767.2 ticks
        try:
            status_l1, self.left_enc1, crcl1 = self.roboclaw[self.left_ind].ReadEncM1(self.left_address)
            status_l2, self.left_enc2, crcl2 = self.roboclaw[self.left_ind].ReadEncM2(self.left_address)
        except ValueError:
            pass
        except OSError as e:
            rospy.logwarn("ReadEnc Left OSError: %d", e.errno)
            rospy.loginfo(e)

        try:
            status_r1, self.right_enc1, crcr1 = self.roboclaw[self.right_ind].ReadEncM1(self.right_address)
            status_r2, self.right_enc2, crcr2 = self.roboclaw[self.right_ind].ReadEncM2(self.right_address)
        except ValueError:
            pass
        except OSError as e:
            rospy.logwarn("ReadEnc Right OSError: %d", e.errno)
            rospy.loginfo(e)
        
        self.encoder_odom.update_publish(self.right_enc1, self.right_enc2, -self.left_enc1, -self.left_enc2)
    

    def print_encoder_data(self):
        print("Left encder M1: %f",self.left_enc1)
        print("Left encder M2: %f",self.left_enc2)
        print("Right encder M1: %f",self.right_enc1)
        print("Right encder M2: %f",self.right_enc2)
        print("------------------------------")

if __name__ == '__main__':
    rospy.init_node('robot_controller')

    robot = MyJay()
    #rospy.spin()
    last_encoder_time = rospy.Time.now().secs # to be used to limit odometry calc rate
    odom_rate = 20.0 #20 hz
    

    rate = rospy.Rate(20) # ROS Rate at 20Hz
    while not rospy.is_shutdown():
        # intake and elevator are supposed to act instantaneously at a single speed
        robot.run_elevator(robot.elevator_pwm)
        robot.run_intake(robot.intake_pwm)


        # run drivetrain at the main loop's rate
        try:
            robot.drive_pwm(robot.drive_motor_pwm[0], robot.drive_motor_pwm[3],
             robot.drive_motor_pwm[1], robot.drive_motor_pwm[2])# front left, front right, back right, back left
        except OSError as e:
            rospy.loginfo("Motor OSError: %d", e.errno)
            rospy.loginfo(e)
        
        # update sensor data
        robot.update_sensor_data()

        rate.sleep()


        # sort of a watchdog
        # if (rospy.get_rostime() - robot.last_cmd_time).to_sec() > 1:
        #     # stop all drive motors so the robot doesnt break stuff (or itself)
        #     robot.stop_drive()

