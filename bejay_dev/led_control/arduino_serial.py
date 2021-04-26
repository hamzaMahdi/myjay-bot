import serial
import time
ser = serial.Serial('/dev/ttyUSB0',9600)
time.sleep(20)
ser.write('1500,0,100,0,0,10\n')
time.sleep(0.1)
ser.close()
