import pygame
import socket
import time
import math
import xbox360_controller
#UDP_IP = "192.168.4.1" # if using ESP32
UDP_IP = "10.42.0.1" # if using jetson nano AP
#UDP_IP = "192.168.0.70"
UDP_PORT = 8080
l1=127/2
l2=127/2
r1=127/2
r2=127/2
intake = 127/2
elevator =  127/2
base_line=127/2
joy_deadzone= 0.2
'''
Joystick setup
'''
pygame.joystick.init()
pygame.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
counter = 0
for joy in joysticks:
    joy_index = counter
    counter+=1

#manually use the first joystick
joy_index = 0
joysticks[joy_index].init()
print("The number of joysticks: " + str(pygame.joystick.get_count()))
print("The name of joystick: " + joysticks[joy_index].get_name())
print("The number of axis: " + str(joysticks[joy_index].get_numaxes()))
hats = joysticks[joy_index].get_numhats()
print ("Number of hats: {}".format(hats) )
'''
end of joystick setup
'''

time.sleep(1)
keepRunning = True

adaptive_controller = xbox360_controller.Controller(0)


def numToStringF(num):
    # Make each pwm value 3 characters
    num = int(num)
    ans = str(num)
    if len(ans) == 1:
        return "00" + ans
    elif len(ans) == 2:
        return "0" + ans
    else:
        return ans

def dpad_control(max_vel, max_angular_rate):
    dpad = adaptive_controller.get_pad()
    axisX = dpad[xbox360_controller.PAD_RIGHT]-dpad[xbox360_controller.PAD_LEFT]
    axisY = dpad[xbox360_controller.PAD_UP]-dpad[xbox360_controller.PAD_DOWN]
    X = max_vel+int(axisX*max_vel*0.5)
    Y = max_vel+int(axisY*max_vel*0.5)
    Z = max_angular_rate#+int(axisS*-max_angular_rate)
    if(X>max_vel*2):
        X=max_vel
    if(Y>max_vel*2):
        Y=max_vel
    if(Z>max_angular_rate*2):
        Z=max_angular_rate

    return X, Y, Z


def joystick_control(max_vel, max_angular_rate):
    left = adaptive_controller.get_left_stick()
    right = adaptive_controller.get_right_stick()
    axisX = right[0]
    axisY = 0#right[1]

    axisS = -right[1]#left[0]

    print('Calculated:')
    print("X: " + str(axisX))
    print("S: " + str(axisS))
    print("Y: " + str(axisY) + "\n\n")
    print('-------')


    X = max_vel+int(axisX*-max_vel)
    Y = max_vel+int(axisY*-max_vel)
    Z = max_angular_rate+int(axisS*-max_angular_rate)
    return X, Y, Z


axisX = 0
axisY = 0
up_button_last = False
down_button_last = False
left_button_last = False
right_button_last = False
while keepRunning:
    if pygame.event.get():
            
            max_vel = 70 #max is 114 cm/s
            max_angular_rate = 3 #rad/s

            X, Y, Z = joystick_control(max_vel, max_angular_rate)#dpad_control(max_vel,max_angular_rate)
            
            buttons  = adaptive_controller.get_buttons()
            intake = int(base_line+buttons[xbox360_controller.A]*60)
            elevator = int(base_line+buttons[xbox360_controller.B]*30)
            flywheel = int(buttons[xbox360_controller.B])

            pygame.time.delay(50)
            pygame.event.pump()
            #time.sleep(0.1)
            MESSAGE = "d" + numToStringF(X) + numToStringF(Y)+numToStringF(Z)+numToStringF(intake)+numToStringF(elevator)+numToStringF(flywheel)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            sock.sendto(MESSAGE.encode("utf-8"), (UDP_IP, UDP_PORT))
            print('the message sent is')
            print(MESSAGE)
            print('-----')
