import pygame
import time

pygame.init()
pygame.joystick.init()
J = pygame.joystick.Joystick(0)
J.init()
print(J.get_name())

while True:
    time.sleep(0.5)
    for k in range(J.get_numaxes()):
        pygame.event.pump()
        print(J.get_axis(k))

    for k in range(J.get_numbuttons()):
        pygame.event.pump()
        print(J.get_button(k))

    for k in range(J.get_numhats()):
        pygame.event.pump()
        print(J.get_hat(k))

