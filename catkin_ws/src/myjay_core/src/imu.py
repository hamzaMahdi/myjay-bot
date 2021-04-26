#!/usr/bin/env python
import rospy
from myjay_core.imu_utils import InitMPU, calibrate, gyro, accel, temp
from sensor_msgs.msg import Imu
from myjay_core.MPU6050 import MPU6050
import math

# this class takes care of getting data from the digital motion processor on the MPU6050
class DMP:
    def __init__(self):
        i2c_bus = 0
        device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        x_accel_offset = -2910
        y_accel_offset = 279
        z_accel_offset = -671
        x_gyro_offset = 14
        y_gyro_offset = 31
        z_gyro_offset = -27
        enable_debug_output = True
        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.packet_size =  self.mpu.DMP_get_FIFO_packet_size()
    
    def get_data(self):
        FIFO_count = self.mpu.get_FIFO_count()
        mpu_int_status = self.mpu.get_int_status()

        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            self.mpu.reset_FIFO()
            # print('overflow!')
            return -1, -1, -1
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < self.packet_size:
                FIFO_count = self.mpu.get_FIFO_count()
            FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
            accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
            quat = self.mpu.DMP_get_quaternion(FIFO_buffer)
            grav = self.mpu.DMP_get_gravity(quat)
            acceleration = self.mpu.DMP_get_linear_accel(accel, grav)
            angular_rates = self.mpu.DMP_get_gyro_int16(FIFO_buffer)
            return quat, acceleration, angular_rates
        else:
            return -1, -1, -1



if __name__ == '__main__':
    rospy.init_node('imu')
    imu_pub = rospy.Publisher('/mpu_imu', Imu, queue_size=10)
    
    # simple init for calibration
    #InitMPU()
    #calibrate() # only accounts for bias

    # init DMP using the MPU6050 class
    dmp = DMP()
    
    # set covariances, TODO: make it a parameter
    pitch_roll_stdev_= 1.0 * (math.pi / 180.0)
    yaw_stdev_= 5.0 * (math.pi / 180.0)
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_
    yaw_covariance = yaw_stdev_ * yaw_stdev_
    
    rate = rospy.Rate(20) # ROS Rate at 20Hz
    
    # main loop
    while not rospy.is_shutdown():
 
        imu_msg = Imu()

        # covariance values calculated by sampling 1000 points and doign np.cov
        imu_msg.linear_acceleration_covariance[0] = 0.22196766
        imu_msg.linear_acceleration_covariance[4] = 0.22206746
        imu_msg.linear_acceleration_covariance[8] = 0.22231805

        imu_msg.angular_velocity_covariance[0] = 5.25367638e-08
        imu_msg.angular_velocity_covariance[4] = 5.59613610e-08 
        imu_msg.angular_velocity_covariance[8] = 5.03703600e-08
        
        imu_msg.orientation_covariance[0] = pitch_roll_covariance 
        imu_msg.orientation_covariance[4] = pitch_roll_covariance 
        imu_msg.orientation_covariance[8] = yaw_covariance 

        #gx, gy, gz = gyro()
        #ax, ay, az = accel()
        quaternion_data, accel_data, angular_rates = dmp.get_data()
        if quaternion_data !=-1:
            imu_msg.orientation.x = quaternion_data.x
            imu_msg.orientation.y = quaternion_data.y
            imu_msg.orientation.z = quaternion_data.z
            imu_msg.orientation.w = quaternion_data.w
            imu_msg.angular_velocity.x = angular_rates.x*131.0*math.pi/180.0
            imu_msg.angular_velocity.y = angular_rates.y*131.0*math.pi/180.0
            imu_msg.angular_velocity.z = angular_rates.z*131.0*math.pi/180.0

            # By default, accel is in arbitrary units with a scale of 16384/1g.
            imu_msg.linear_acceleration.x = accel_data.x * 1/16384. * 9.80665
            imu_msg.linear_acceleration.y = accel_data.y * 1/16384. * 9.80665
            imu_msg.linear_acceleration.z = accel_data.z * 1/16384. * 9.80665

            imu_msg.header.frame_id = 'imu'
            imu_msg.header.stamp = rospy.Time.now()

            imu_pub.publish(imu_msg)

        rate.sleep()

