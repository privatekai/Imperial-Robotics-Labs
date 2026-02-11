import numpy as np
import time

# square constants
# Ensure these are abstracted away from the rest of the code
SQUARE_X_OFFSET = 200 # adjust these two to position the square
SQUARE_Y_OFFSET = 100
SQUARE_DRAW_SIZE = 500 # adjust this to scale the square
SQUARE_REAL_SIZE = 400 # in mm

# particle constants
NUM_PARTICLES = 100
ROBOT_START_POS = (0, 0, 0)
SCREEN_ORIGIN = (SQUARE_X_OFFSET, SQUARE_DRAW_SIZE + SQUARE_Y_OFFSET)

# distribution constants
E_MEAN, E_VAR = 0, 10 # in mm
F_MEAN, F_VAR = 0, 1 # in degrees
G_MEAN, G_VAR = 0, 1 # in degrees

"""
Helper functions!
"""

def real_to_screen(particle):
    """
    Converts the robot's world coordinates to the drawn screen coordinates.
    """
    x, y, theta = particle
    x = ((x / SQUARE_REAL_SIZE) * SQUARE_DRAW_SIZE) + SQUARE_X_OFFSET
    y = ((y / SQUARE_REAL_SIZE) * SQUARE_DRAW_SIZE) + SQUARE_Y_OFFSET + SQUARE_DRAW_SIZE
    return (x, y, theta)

def robot_position(particles, weights):
    """
    Takes the average of the world coordinate particles to give an estimate of the robot's real coordinates.
    """
    x, y, theta = (0, 0, 0)
    for i in range(NUM_PARTICLES):
        x += particles[i][0] * weights[i]
        y += particles[i][1] * weights[i]
        theta += particles[i][2] * weights[i]

    particle = (x, y, theta)
    return particle

def print_formatted_robot_pos(particles, screen_particles=None):
        x, y, theta = robot_position(particles, np.array([1/NUM_PARTICLES] * NUM_PARTICLES))
        print(f"robot_position: ({x:.2f}, {y:.2f}, {theta:.2f})")
        if screen_particles is not None:
            sx, sy, stheta = robot_position(screen_particles, np.array([1/NUM_PARTICLES] * NUM_PARTICLES))
            print(f"screen robot_position: ({sx:.2f}, {sy:.2f}, {stheta:.2f})\n")

def print_draw_particles(particles):
    screen_particles = np.apply_along_axis(lambda p: np.array(real_to_screen(p)), axis=1, arr=particles)
    print("drawParticles:", list(map(tuple, screen_particles)))

"""
Movement Functions!
"""

def apply_forward(particle, distance):
    """
    particle: the particle to to move forward
    distance: the distance (in cm) that the robot moves forward
    """    
    x, y, theta = particle

    angle = np.deg2rad(theta)
    x_rand = np.random.normal(E_MEAN, E_VAR)
    y_rand = np.random.normal(E_MEAN, E_VAR)
    theta_rand = np.random.normal(F_MEAN, F_VAR)
    
    x_new = x + (distance + x_rand) * np.cos(angle)
    y_new = y + (distance + y_rand) * np.sin(angle)
    theta_new = theta + theta_rand

    return np.array([x_new, y_new, theta_new])

def apply_turn(particle, angle):
    """
    particle: the particle to to move forward
    distance: the angle (in degrees) that the robot rotates
    """
    x, y, theta = particle
    theta_rand = np.random.normal(G_MEAN, G_VAR)

    return np.array([x, y, theta + angle + theta_rand])

def apply_all_forward(particles, distance):
    particles = np.apply_along_axis(lambda p: apply_forward(p, distance), axis=1, arr=particles)
    screen_particles = np.apply_along_axis(lambda p: np.array(real_to_screen(p)), axis=1, arr=particles)

    print_draw_particles(particles)
    print_formatted_robot_pos(particles, screen_particles)

    return particles

def apply_all_turn(particles, angle):
    particles = np.apply_along_axis(lambda p: apply_turn(p, angle), axis=1, arr=particles)
    screen_particles = np.apply_along_axis(lambda p: np.array(real_to_screen(p)), axis=1, arr=particles)

    print_formatted_robot_pos(particles, screen_particles)

    return particles

def initial_drawing(particles):
    corners = [(SQUARE_X_OFFSET, SQUARE_Y_OFFSET), 
               (SQUARE_DRAW_SIZE + SQUARE_X_OFFSET, SQUARE_Y_OFFSET), 
               (SQUARE_DRAW_SIZE + SQUARE_X_OFFSET, SQUARE_DRAW_SIZE + SQUARE_Y_OFFSET), (SQUARE_X_OFFSET, SQUARE_DRAW_SIZE + SQUARE_Y_OFFSET)]
    lines = []
    for i in range(1, len(corners) + 1):
        lines.append(corners[i - 1] + corners[i % len(corners)])

    for line in lines:
        print("drawLine:", str(line))

    time.sleep(1)

    print("drawParticles:", list(map(tuple, particles)))

"""
Main body
"""

# particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)

# initial_drawing(particles)
# print_formatted_robot_pos(particles)

# for _ in range(4):
#     for _ in range(4):
#         time.sleep(1)
#         print("Moving forward 100")
#         particles = apply_all_forward(particles, 100)
#     print("Turning -90 degrees")
#     particles = apply_all_turn(particles, -90)