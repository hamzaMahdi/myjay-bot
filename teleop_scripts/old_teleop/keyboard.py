import pygame
import socket
import time
import math
import pygame
UDP_IP = "192.168.4.1"
UDP_PORT = 8080
F=255
H=255
pygame.init()
#pygame.display.set_mode()
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
while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit(); #sys.exit() if sys is imported
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_LEFT:
				H = 400
			elif event.key == pygame.K_RIGHT:
				H=100
			else:
				H=255
			if event.key == pygame.K_UP:
				F=100
			elif event.key == pygame.K_DOWN:
				F=400
			else:
				F=255
		else:
			F=255
			H=255
	MESSAGE = MESSAGE = "d" + numToStringF(F) + numToStringF(H)+"255"
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
	sock.sendto(MESSAGE.encode("utf-8"), (UDP_IP, UDP_PORT))
	print(MESSAGE)
	time.sleep(0.5)
