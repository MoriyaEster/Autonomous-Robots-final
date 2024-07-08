import random
import pygame

def add_noise(value, epsilon):
    return value + random.uniform(-epsilon, epsilon)

def rotate_point(point, angle, origin=(0, 0)):
    from math import radians, cos, sin
    angle = radians(angle)
    ox, oy = origin
    px, py = point

    qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
    qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
    return qx, qy

def load_map(file_path):
    import pygame
    return pygame.image.load(file_path)

def draw_drone(screen, position, color=(0, 255, 0), radius=5):
    pygame.draw.circle(screen, color, position, radius)

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

def draw_charging_message(screen):
    font = pygame.font.Font(None, 30)
    text = font.render('Charging', True, (255, 0, 0))
    screen.blit(text, (100, 500))