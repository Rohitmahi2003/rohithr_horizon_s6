import pygame
import random
import math

pygame.init()

# Set up display
win = pygame.display.set_mode((500, 500))
pygame.display.set_caption("Interactive Circle Drawing")

# Create a persistent background surface
background = pygame.Surface(win.get_size())
background.fill((255, 255, 255))  # Initial white background

# Initialize variables
circles = []  # List to store circles (x, y, radius, color)
holding = False
start_pos = None
radius = 5
path_length = 0

run = True
while run:
    win.blit(background, (0, 0))  # Load previous drawings without clearing everything

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        # Left-click starts drawing
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            start_pos = event.pos
            holding = True
            radius = 10  # Reset radius for new circle

        # Stop growing when released
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if start_pos:
                color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                circles.append((start_pos[0], start_pos[1], radius, color))

                # Draw the circle permanently onto the background
                pygame.draw.circle(background, color, (start_pos[0], start_pos[1]), radius)

                # Update path length if there's a previous circle
                if len(circles) > 1:
                    prev_x, prev_y, _, _ = circles[-2]
                    new_x, new_y, _, _ = circles[-1]
                    path_length += math.sqrt((new_x - prev_x) ** 2 + (new_y - prev_y) ** 2)
                    pygame.draw.line(background, (0, 0, 0), (prev_x, prev_y), (new_x, new_y), 2)

                holding = False

        # Right-click to remove last circle
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            if circles:
                removed_circle = circles.pop()

                # Redraw everything on background
                background.fill((255, 255, 255))  # Clear background
                for i, (x, y, r, color) in enumerate(circles):
                    pygame.draw.circle(background, color, (x, y), r)
                    if i > 0:
                        pygame.draw.line(background, (0, 0, 0), (circles[i-1][0], circles[i-1][1]), (x, y), 2)

                # Update path length
                path_length = sum(
                    math.sqrt((circles[i][0] - circles[i-1][0]) ** 2 + (circles[i][1] - circles[i-1][1]) ** 2)
                    for i in range(1, len(circles))
                )

        # 'S' key to save image
        if event.type == pygame.KEYDOWN and event.key == pygame.K_s:
            pygame.image.save(background, "drawing.png")
            print("Image saved!")

        # Spacebar to clear everything
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            background.fill((255, 255, 255))  # Reset background
            circles.clear()
            path_length = 0
            print("Canvas cleared!")

    # Increase circle size while holding left-click
    if holding and start_pos:
        radius += 1  
        pygame.draw.circle(win, (0, 0, 0), start_pos, radius, 1)  # Draw temporary growing circle

    # Display path length
    font = pygame.font.Font(None, 30)
    text = font.render(f"Path Length: {round(path_length, 2)}", True, (0, 0, 0))
    win.blit(text, (10, 10))

    pygame.display.update()

pygame.quit()
