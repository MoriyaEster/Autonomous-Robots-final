import pygame
import time
from PIL import Image
from logic import Drone, image_to_matrix, build_track_from_matrix, find_closest_track_point

image_path = "p11.png"
img = Image.open(image_path).convert('L')  # Convert to grayscale
matrix = image_to_matrix(img)

WINDOW_WIDTH, WINDOW_HEIGHT = img.width, img.height
BACKGROUND_COLOR = (0, 0, 0)
DRONE_COLOR = (255, 0, 0)
TRACK_COLOR = (50, 50, 50)
HISTORY_COLOR = (255, 255, 0)
SENSOR_COLOR = (255, 255, 0)
PATH_COLOR = (0, 255, 0)  # Green color for the path

VELOCITY = 2
DRONE_RADIUS = 10  # 10 centimeters

pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption('3D Modeling with Drone Sensors')


def draw_track(screen, track, track_color):
    for segment in track:
        pygame.draw.rect(screen, track_color, segment)


def main():
    matrix = image_to_matrix(img)
    cell_size = 1

    track = build_track_from_matrix(matrix, cell_size)
    start_x, start_y = 125, 125

    initial_rect = pygame.Rect(start_x, start_y, 1, 1)
    if not any(segment.colliderect(initial_rect) for segment in track):
        start_x, start_y = find_closest_track_point(start_x, start_y, track)

    drone = Drone(start_x, start_y, VELOCITY, DRONE_RADIUS, matrix, WINDOW_WIDTH, WINDOW_HEIGHT)

    clock = pygame.time.Clock()
    running = True

    start_time = time.time()

    while running:
        screen.fill(BACKGROUND_COLOR)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        drone.move()
        draw_track(screen, track, TRACK_COLOR)
        drone.draw_history(screen, HISTORY_COLOR)
        drone.draw_sensors(screen, SENSOR_COLOR)
        drone.draw_path(screen, PATH_COLOR)  # Draw the path
        drone.draw(screen, DRONE_COLOR)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
