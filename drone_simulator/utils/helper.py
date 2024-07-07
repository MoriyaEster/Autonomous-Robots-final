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

def draw_battery_dead_message(screen):
    font = pygame.font.Font(None, 30)
    text = font.render('Battery Died', True, (255, 0, 0))
    screen.blit(text, (100, 500))