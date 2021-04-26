import pygame

import xbox360_controller
pygame.init()
my_controller = xbox360_controller.Controller(0)



done = False
while not done:
    # event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done=True
        print(my_controller.get_buttons())
        print("----")
        print(my_controller.get_pad())
        print(my_controller.get_triggers())

        print("-------------")
