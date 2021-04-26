import pygame
import socket
import time
import math
#UDP_IP = "192.168.4.1" # if using ESP32
UDP_IP = "10.42.0.1" # if using jetson nano AP
#UDP_IP = "192.168.0.70"
UDP_PORT = 8090
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
joy_index = 1
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

axisX = 0
axisY = 0
up_button_last = False
down_button_last = False
left_button_last = False
right_button_last = False
while keepRunning:
    if pygame.event.get():
            #time.sleep(1)
            if abs(round(joysticks[joy_index].get_axis(2), 1))>joy_deadzone:
                axisY =  (round(joysticks[joy_index].get_axis(2), 1))
            else:
                axisY=0
            if abs(round(joysticks[joy_index].get_axis(3), 1))>joy_deadzone:
                axisX = round(joysticks[joy_index].get_axis(3), 1)
            else:
                axisX=0
            if abs(round(joysticks[joy_index].get_axis(0), 1))>joy_deadzone:
                axisS =  round(joysticks[joy_index].get_axis(0), 1) 
            else:
                axisS=0
            print('Calculated:')
            print("X: " + str(axisX))
            print("S: " + str(axisS))
            print("Y: " + str(axisY) + "\n\n")
            print('-------')
            print('joystick axis:')
            print(round(joysticks[joy_index].get_axis(3), 1))
            print(round(joysticks[joy_index].get_axis(1), 1))
            print('-------')

            max_vel = 70 #max is 114 cm/s
            max_angular_rate = 3 #rad/s
            X = max_vel+int(axisX*-max_vel)
            Y = max_vel+int(axisY*-max_vel)
            Z = max_angular_rate+int(axisS*-max_angular_rate)
            if(X>max_vel*2):
                X=max_vel


            if(Y>max_vel*2):
                Y=max_vel


            if(Z>max_angular_rate*2):
                Z=max_angular_rate
       
            intake = int(base_line+(joysticks[joy_index].get_button(8)+joysticks[joy_index].get_button(11))*60)
            elevator = int(base_line+(joysticks[joy_index].get_button(9)+joysticks[joy_index].get_button(11))*30)
            flywheel = int(joysticks[joy_index].get_button(13))+int(joysticks[joy_index].get_button(11))

            pygame.time.delay(50)
            pygame.event.pump()
            #time.sleep(0.1)
            MESSAGE = "d" + numToStringF(X) + numToStringF(Y)+numToStringF(Z)+numToStringF(intake)+numToStringF(elevator)+numToStringF(flywheel)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            sock.sendto(MESSAGE.encode("utf-8"), (UDP_IP, UDP_PORT))
            print('the message sent is')
            print(MESSAGE)
            print('-----')
