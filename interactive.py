import pygame
import random

pygame.init()

# Set up display
win = pygame.display.set_mode((500, 500))
pygame.display.set_caption("Interactive Canvas")

background = pygame.Surface(win.get_size())
background.fill((255, 255, 255))

holding = False
start_pos = None
radius = 5
circles = []

run = True
while run:
    win.blit(background, (0, 0))
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            start_pos = event.pos
            holding = True
            radius = 10

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if start_pos:
                color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                circles.append((start_pos[0], start_pos[1], radius, color))
                pygame.draw.circle(background, color, start_pos, radius)
                holding = False

    if holding and start_pos:
        radius += .1  
        pygame.draw.circle(win, (0, 0, 0), start_pos, radius, 1)
    
    pygame.display.update()

pygame.quit()
