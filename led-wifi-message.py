import pygame
import socket
import time
import math
import pygame
UDP_IP = "192.168.4.1"
UDP_PORT = 8080

pygame.init()
MESSAGE = ""
while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit(); 

		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_5:
				MESSAGE = "solid"
			elif event.key == pygame.K_2:
				MESSAGE = "rainbow"
			elif event.key == pygame.K_1:
				MESSAGE = "wakeup"
			elif event.key == pygame.K_3:
				MESSAGE = "forward"
			elif event.key == pygame.K_4:
				MESSAGE = "backward"
		
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
	sock.sendto(MESSAGE.encode("utf-8"), (UDP_IP, UDP_PORT))

	time.sleep(0.5)
