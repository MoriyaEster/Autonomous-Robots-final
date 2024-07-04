import pygame

WINDOW_WIDTH, WINDOW_HEIGHT = 800, 600
BACKGROUND_COLOR = (0, 0, 0)
DRONE_COLOR = (255, 0, 0)
TRACK_COLOR = (50, 50, 50)
HISTORY_COLOR = (255, 255, 0)
SENSOR_COLOR = (255, 255, 0)
PATH_COLOR = (0, 255, 0)

VELOCITY = 2
DRONE_RADIUS = (int)(3000 / 2.5)

pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption('3D Modeling with Drone Sensors')

def draw_track(screen, track, track_color):
    for segment in track:
        pygame.draw.rect(screen, track_color, segment)

def handle_input(drone, keys):
    if keys[pygame.K_LEFT]:
        drone.direction_index = 1
    if keys[pygame.K_RIGHT]:
        drone.direction_index = 0
    if keys[pygame.K_UP]:
        drone.direction_index = 2
    if keys[pygame.K_DOWN]:
        drone.direction_index = 3
    drone.current_direction = drone.directions[drone.direction_index]
