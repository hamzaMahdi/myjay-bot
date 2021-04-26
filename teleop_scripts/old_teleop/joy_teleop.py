import pygame
import socket
import time
import math
#UDP_IP = "192.168.4.1" # if using ESP32
UDP_IP = "10.42.0.1" # if using jetson nano AP
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
axisX = 0;
axisY = 0;
X = 25;
up_button_last = False
down_button_last = False
left_button_last = False
right_button_last = False
while keepRunning:
    #for event in pygame.event.get():
            #time.sleep(1)
            if abs(round(joysticks[joy_index].get_axis(3), 1))>joy_deadzone:
                axisY =  (round(joysticks[joy_index].get_axis(3), 1)) * -base_line
            else:
                axisY=0
            if abs(round(joysticks[joy_index].get_axis(0), 1))>joy_deadzone:
                axisX = round(joysticks[joy_index].get_axis(0), 1) * base_line
            else:
                axisX=0
            if abs(round(joysticks[joy_index].get_axis(2), 1))>joy_deadzone:
                axisS =  round(joysticks[joy_index].get_axis(2), 1) * -base_line
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
            l1 = int(base_line+axisY+axisX-axisS)
            l2 = int(base_line+-axisY-axisX-axisS)
            r1 = int(base_line+axisY-axisX+axisS)
            r2 = int(base_line+-axisY+axisX+axisS)
            if(l1>127):
            	l1=127
            elif(l1<0):
            	l1=0
            if(l2>127):
            	l2=127
            elif(l2<0):
            	l2=0
            if(r1>127):
            	r1=127
            elif(r1<0):
            	r1=0
            if(r2>127):
            	r2=127
            elif(r2<0):
            	r2=0
            intake = int(base_line+joysticks[joy_index].get_button(14)*30)
            elevator = int(base_line+joysticks[joy_index].get_button(14)*30)
            flywheel = int(joysticks[joy_index].get_button(13))
            print('the motors in sequence (l1,l2,r1,r2) are:')
            print((l1,l2,r1,r2))
            pygame.time.delay(50)
            pygame.event.pump()
            #time.sleep(0.1)
            MESSAGE = "d" + numToStringF(l1) + numToStringF(l2)+numToStringF(r1)+numToStringF(r2)+numToStringF(intake)+numToStringF(elevator)+numToStringF(flywheel)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            sock.sendto(MESSAGE.encode("utf-8"), (UDP_IP, UDP_PORT))
            print('the message sent is')
            print(MESSAGE)
            print('-----')
