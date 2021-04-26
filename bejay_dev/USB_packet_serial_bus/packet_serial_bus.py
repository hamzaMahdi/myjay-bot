from roboclaw import Roboclaw
from time import sleep


# this depends on the physical connections in your robot
right_address = 0x80
left_address = 0x82
intake_address = 0x81
addresses = [right_address,left_address,intake_address]
# it's better to create a udev rule with specific names but they all have the same attributes
roboclaw = []
roboclaw.append(Roboclaw("/dev/ttyACM0", 38400))
roboclaw.append(Roboclaw("/dev/ttyACM1", 38400))
roboclaw.append(Roboclaw("/dev/ttyACM2", 38400))


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
print('Battery Voltage Level:')
print(roboclaw[right_ind].ReadMainBatteryVoltage(right_address))
'''
while True:
    print(roboclaw[right_ind].ReadSpeedM1(right_address))
    print(roboclaw[right_ind].ReadSpeedM2(right_address))
    sleep(0.2)
'''
