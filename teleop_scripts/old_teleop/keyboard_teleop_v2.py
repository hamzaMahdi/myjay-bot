import pygame
import socket
import time
import math
import pygame
UDP_IP = "192.168.4.1"
UDP_PORT = 8080
l1=127/2
l2=127/2
r1=127/2
r2=127/2
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
				l1 = 127/2-40
				l2 = 127/2+40
				r1 = 127/2+40
				r2 = 127/2-40
			elif event.key == pygame.K_RIGHT:
				l1 = 127/2+40
				l2 = 127/2-40
				r1 = 127/2-40
				r2 = 127/2+40
			elif event.key == pygame.K_a:
				l1 = 127/2+40
				l2 = 127/2+40
				r1 = 127/2-40
				r2 = 127/2-40
			elif event.key == pygame.K_d:
				l1 = 127/2-40
				l2 = 127/2-40
				r1 = 127/2+40
				r2 = 127/2+40
			elif event.key == pygame.K_UP:
				l1 = 127/2+40
				l2 = 127/2-40
				r1 = 127/2+40
				r2 = 127/2-40
			elif event.key == pygame.K_DOWN:
				l1 = 127/2-40
				l2 = 127/2+40
				r1 = 127/2-40
				r2 = 127/2+40
			else:
				l1=127/2
				l2=127/2
				r1=127/2
				r2=127/2
		else:
			l1=127/2
			l2=127/2
			r1=127/2
			r2=127/2
	MESSAGE = MESSAGE = "d" + numToStringF(l1) + numToStringF(l2)+numToStringF(r1)+numToStringF(r2)+'090'+'100'#+"255"
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
	sock.sendto(MESSAGE.encode("utf-8"), (UDP_IP, UDP_PORT))
	print(MESSAGE)
	time.sleep(0.3)
