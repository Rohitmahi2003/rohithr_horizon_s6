import pygame
import math

pygame.init()

win = pygame.display.set_mode((500, 500))
pygame.display.set_caption("Path Creation")

background = pygame.Surface(win.get_size())
background.fill((255, 255, 255))

circles = []
run = True

while run:
    win.blit(background, (0, 0))
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = event.pos
            circles.append(pos)
            pygame.draw.circle(background, (0, 0, 255), pos, 5)
            
            if len(circles) > 1:
                pygame.draw.line(background, (0, 0, 0), circles[-2], circles[-1], 2)
                
    total_length = 0
    for i in range(1, len(circles)):
        total_length += math.dist(circles[i-1], circles[i])
    
    pygame.display.set_caption(f"Path Creation - Total Length: {total_length:.2f}")
    pygame.display.update()

pygame.quit()
