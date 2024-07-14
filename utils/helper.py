import random
import pygame


def draw_battery_bar(screen, remaining_time, total_time, position=(10, 10), size=(100, 20), color=(0, 255, 0)):
    if total_time == 0:
        return

    battery_percentage = remaining_time / total_time
    battery_width = int(size[0] * battery_percentage)
    battery_rect = pygame.Rect(position, (battery_width, size[1]))
    border_rect = pygame.Rect(position, size)

    pygame.draw.rect(screen, color, battery_rect)
    pygame.draw.rect(screen, (255, 255, 255), border_rect, 2)

def draw_battery_dead_message(screen):
    font = pygame.font.Font(None, 30)
    text = font.render('Battery Died', True, (255, 0, 0))
    screen.blit(text, (100, 500))

def draw_battery_low_message(screen):
    font = pygame.font.Font(None, 30)
    text = font.render('Battery low', True, (255, 0, 0))
    screen.blit(text, (100, 500))
