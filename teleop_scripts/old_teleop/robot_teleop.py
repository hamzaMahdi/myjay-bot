#!/usr/bin/env python

from socket import *
import sys
import select
from roboclaw import Roboclaw
from time import sleep
from time import time

# networking parameters
host="10.42.0.1"
port = 8080

# flywheel parameters
acceleration = 0.1
flywheel = 1500 # neutral PWM value 

# this depends on the physical connections in your robot
right_address = 0x80
left_address = 0x82
intake_address = 0x81
addresses = [right_address,left_address,intake_address]
# it's better to create a udev rule with specific names but they all have the s$
roboclaw = []
roboclaw.append(Roboclaw("/dev/ttyACM0", 38400))
roboclaw.append(Roboclaw("/dev/ttyACM1", 38400))
roboclaw.append(Roboclaw("/dev/ttyACM2", 38400))

def setup_motors():

	print('setting up connections...')
	for rc_ind, rc in enumerate(roboclaw):
		rc.Open()
		for add_ind, address in enumerate(addresses):
			if(rc.ReadNVM(address) and address==right_address):
				right_ind = rc_ind
				break
			if(rc.ReadNVM(address) and address==intake_address):
				intake_ind = rc_ind
				break
			if(rc.ReadNVM(address) and address==left_address):
				left_ind = rc_ind
				break
	print('setup complete !')
	return right_ind, intake_ind, left_ind

def drive(l1, l2, r1, r2):
	roboclaw[left_ind].ForwardBackwardM1(left_address, l1)
	roboclaw[left_ind].ForwardBackwardM2(left_address, l2)
	roboclaw[right_ind].ForwardBackwardM2(right_address, r1)
	roboclaw[right_ind].ForwardBackwardM1(right_address, r2)
	if l1>l2:# going forward
		return 3,(0,100,0)
	elif l1<l2:# going back
		return 4
	else:# still or sideways
		return 0

def run_intake(Mspeed):
	roboclaw[intake_ind].ForwardBackwardM1(intake_address, Mspeed)

def run_elevator(Mspeed):
	roboclaw[intake_ind].ForwardBackwardM2(intake_address, Mspeed)


def stop_drive():
	roboclaw[left_ind].ForwardM1(left_address, 0)
	roboclaw[left_ind].ForwardM2(left_address, 0)
	roboclaw[right_ind].ForwardM1(right_address, 0)
	roboclaw[right_ind].ForwardM2(right_address, 0)




right_ind, intake_ind, left_ind = setup_motors()



s = socket(AF_INET,SOCK_DGRAM)
s.bind((host,port))

addr = (host,port)
buf=1024


while True and roboclaw[right_ind].ReadMainBatteryVoltage(right_address)[1]>100:#as long as the battery voltage is above 10 V keep going
	data,addr = s.recvfrom(buf)
	print "Received File:",data.strip()
	print addr
	last_seen = time()
	msg = []
	message_size=7 # n_commands sent,probably wont change
	for i in range(message_size):
		msg.append(int(data.strip()[i*3+1:4+i*3]))
	led_state, color = drive(msg[0],msg[1],msg[2],msg[3])
	run_intake(msg[4])
	run_elevator(msg[5])
	if(msg[6]):#flywheel
		flywheel-=acceleration # negative because throw is <1500 uS
	elif flwheel<1500:
		flywheel+=acceleration
	
	
	


stop_drive()







