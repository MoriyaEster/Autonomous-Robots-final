import time

import pygame
from PIL import Image
from logic import Drone, image_to_matrix, build_track_from_matrix, find_closest_track_point
from ui import draw_track, handle_input

image_path = "p11.png"
img = Image.open(image_path).convert('L')  # Convert to grayscale
matrix = image_to_matrix(img)

WINDOW_WIDTH, WINDOW_HEIGHT = img.width, img.height
BACKGROUND_COLOR = (0, 0, 0)
DRONE_COLOR = (255, 0, 0)
TRACK_COLOR = (50, 50, 50)
HISTORY_COLOR = (255, 255, 0)

VELOCITY = 2
DRONE_RADIUS = (int)(3000 / 2.5)

pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption('3D Modeling with Drone Sensors')


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

    steps = []

    start_time = time.time()

    while running:
        start_time_running = time.time()
        screen.fill(BACKGROUND_COLOR)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if (time.time() - start_time) > 8 * 60 or (drone.x == start_x and drone.y == start_y and drone.is_in_return):
            break
        elif (time.time() - start_time) > 4 * 60:
            drone.is_in_return = True
            drone.calculate_directions_step(5)

            if steps:
                func = steps.pop()
                func(drone)
        else:
            keys = pygame.key.get_pressed()
            handle_input(drone, keys)

            current_readings = drone.get_sensor_readings(drone.directions)

            if (current_readings[2] > 1) and drone.is_going_up:
                drone.is_going_right = True
                drone.is_going_left = True
                drone.move(0, (-1) * VELOCITY)
                steps.append(lambda d: d.move(0, 1 * VELOCITY))
            elif (current_readings[0] > 1) and drone.is_going_right:
                drone.is_going_up = True
                drone.move(1 * VELOCITY, 0)
                steps.append(lambda d: d.move((-1) * VELOCITY, 0))
            elif (current_readings[3] > 1) and drone.is_going_down:
                drone.is_going_right = True
                drone.is_going_up = False
                drone.move(0, 1 * VELOCITY)
                steps.append(lambda d: d.move(0, (-1) * VELOCITY))
            elif (current_readings[1] > 1) and drone.is_going_left:
                drone.is_going_right = False
                drone.is_going_down = True
                drone.is_going_up = False
                drone.move((-1) * VELOCITY, 0)
                steps.append(lambda d: d.move(1 * VELOCITY, 0))
            else:
                # Handle corner cases
                if not drone.is_going_up and not drone.is_going_down and not drone.is_going_right and not drone.is_going_left:
                    # Check surroundings and choose an alternative path
                    if current_readings[0] > 1:
                        drone.move(1 * VELOCITY, 0)
                        steps.append(lambda d: d.move((-1) * VELOCITY, 0))
                    elif current_readings[1] > 1:
                        drone.move((-1) * VELOCITY, 0)
                        steps.append(lambda d: d.move(1 * VELOCITY, 0))
                    elif current_readings[2] > 1:
                        drone.move(0, (-1) * VELOCITY)
                        steps.append(lambda d: d.move(0, 1 * VELOCITY))
                    elif current_readings[3] > 1:
                        drone.move(0, 1 * VELOCITY)
                        steps.append(lambda d: d.move(0, (-1) * VELOCITY))
                else:
                    drone.is_going_up = False
                    drone.is_going_down = False
                    drone.is_going_right = False
                    drone.is_going_left = True
                    drone.move(0, (-1) * VELOCITY)
                    steps.append(lambda d: d.move(0, 1 * VELOCITY))

        draw_track(screen, track, TRACK_COLOR)
        drone.draw_history(screen, HISTORY_COLOR)
        drone.draw(screen, DRONE_COLOR)

        readings = current_readings
        print(f"right: {readings[0]}, left: {readings[1]}, up: {readings[2]}, down: {readings[3]}")

        pygame.display.flip()
        clock.tick(300)
        print(f"time in running = {time.time() - start_time_running}")
        print(f"speed = {VELOCITY / (time.time() - start_time_running)}")

    pygame.quit()


if __name__ == "__main__":
    main()
