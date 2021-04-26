import smbus
import time
 
PWR_M   = 0x6B
DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_EN   = 0x38
ACCEL_X = 0x3B
ACCEL_Y = 0x3D
ACCEL_Z = 0x3F
GYRO_X  = 0x43
GYRO_Y  = 0x45
GYRO_Z  = 0x47
TEMP = 0x41
bus = smbus.SMBus(0)
Device_Address = 0x68   # device address
 
AxCal=0
AyCal=0
AzCal=0
GxCal=0
GyCal=0
GzCal=0

debug = 0

def InitMPU():
    bus.write_byte_data(Device_Address, DIV, 7)
    bus.write_byte_data(Device_Address, PWR_M, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_EN, 1)
    time.sleep(1)
 
def readMPU(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value - 65536
    return value
def accel():
    x = readMPU(ACCEL_X)
    y = readMPU(ACCEL_Y)
    z = readMPU(ACCEL_Z)
    
    Ax = (x/16384.0-AxCal) 
    Ay = (y/16384.0-AyCal) 
    Az = (z/16384.0-AzCal)+1
    if debug:
        print('accels:')
        print ("ACCEL X scaled: ", Ax)
        print ("ACCEL Y scaled: ", Ay)
        print ("ACCEL Z scaled: ", Az)
    return Ax, Ay, Az

 
def gyro():
    global GxCal
    global GyCal
    global GzCal
    x = readMPU(GYRO_X)
    y = readMPU(GYRO_Y)
    z = readMPU(GYRO_Z)
    Gx = x/131.0 - GxCal
    Gy = y/131.0 - GyCal
    Gz = z/131.0 - GzCal
    if debug:
        print('gyro (deg/s):')
        print ("Gyro X scaled: ", Gx)
        print ("Gyro Y scaled: ", Gy)
        print ("Gyro Z scaled: ", Gz)
    return Gx, Gy, Gz

def temp():
    tempRow=readMPU(TEMP)
    tempC=(tempRow / 340.0) + 36.53
    if debug:
        tempC="%.2f" %tempC
        print (tempC)
    return tempC

 
def calibrate():
    print("Calibrate....")
    global AxCal
    global AyCal
    global AzCal
    x=0
    y=0
    z=0
    for i in range(50):
        x = x + readMPU(ACCEL_X)
        y = y + readMPU(ACCEL_Y)
        z = z + readMPU(ACCEL_Z)
    x= x/50
    y= y/50
    z= z/50
    AxCal = x/16384.0
    AyCal = y/16384.0
    AzCal = z/16384.0

    print AxCal
    print AyCal
    print AzCal

    global GxCal
    global GyCal
    global GzCal
    x=0
    y=0
    z=0
    for i in range(50):
        x = x + readMPU(GYRO_X)
        y = y + readMPU(GYRO_Y)
        z = z + readMPU(GYRO_Z)
    x= x/50
    y= y/50
    z= z/50
    GxCal = x/131.0
    GyCal = y/131.0
    GzCal = z/131.0

    print GxCal
    print GyCal
    print GzCal
 
 
InitMPU()
calibrate()
while 1:
    gyro()
    accel()
    temp()
    time.sleep(1)
     