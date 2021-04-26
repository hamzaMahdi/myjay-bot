# Simple test program for arming and moving an ESC using the adafruit PCA9685 
import time

# Import the PCA9685 module.
# https://github.com/adafruit/Adafruit_Python_PCA9685
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40) as seen on the Jetson Hacks demo
# Create the I2C bus interface on the Jetson Nano
pwm = Adafruit_PCA9685.PCA9685(address=0x40,busnum=1) 


'''
If 50 Hz is required
1.0ms/20ms*4096 = 204 full reverse
1.5ms/20ms*4096 = 307 neutral
2.0ms/20ms*4096 = 409 full forward
'''

# Here we will be using 60 Hz
esc_min = 245  # Min pulse length out of 4096
esc_max = 491  # Max pulse length out of 4096
esc_neutral = 375 # should be 368 but my ESC needs calibration i guess


# Set frequency to 60hz, good for servos.
print('arming...')
pwm.set_pwm_freq(60)
pwm.set_pwm(0, 0, esc_neutral)
time.sleep(7)
print('Moving servo on channel 0, press Ctrl-C to quit...')


while True:
    pulse_length = int(input('enter pulse length\n'))
    pwm.set_pwm(0, 0, pulse_length)
