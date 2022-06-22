import pygame

pygame.init()

window = pygame.display.set_mode((600, 400))
pygame.display.set_caption("Grafo aleatorio")

# Variable globales de personaje (nodo)
playerX = 50
playerY = 50
playerWidth = 30
playerHeight = 30

window.fill((255, 255, 255))
running = True
while running:
    for event in pygame.event.get():
        print(event)
        if event.type == pygame.QUIT:
            running = False
    pygame.draw.circle(window, (0, 0, 255), (playerX, playerY), playerWidth, 0)
    pygame.display.flip()


pygame.display.quit()
pygame.quit()
