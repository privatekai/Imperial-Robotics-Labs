import numpy as np
from questions3 import forward, turnClockwise
from visualisation import NUM_PARTICLES, SQUARE_DRAW_SIZE, SQUARE_REAL_SIZE, SQUARE_X_OFFSET, SQUARE_Y_OFFSET

def robot_position(particles, weights):
    x, y, theta = 0
    for i in range(NUM_PARTICLES):
        x += particles[i][0] * weights[i]
        y += particles[i][1] * weights[i]
        theta += particles[i][2] * weights[i]

    x /= NUM_PARTICLES
    y /= NUM_PARTICLES
    theta /= NUM_PARTICLES
    return screen_to_real(x, y, theta)

def screen_to_real(particle):
    x, y, theta = particle
    x = (x - SQUARE_X_OFFSET / SQUARE_DRAW_SIZE) * SQUARE_REAL_SIZE
    y = (y - SQUARE_Y_OFFSET / SQUARE_DRAW_SIZE) * SQUARE_REAL_SIZE
    return (x, y, theta)

def navigate_to_waypoint(waypoint, particles, weights):
    # mean of the particles X, Y and theta
    # convert mean from draw coordinates to real coordinates
    # find angle and distance to get to waypoint
    # move robot (this will update the particles).
    robot_x, robot_y, robot_theta = robot_position(particles, weights)
    w_x, w_y = waypoint

    distance = np.sqrt((w_x - robot_x)^2 + (w_y - robot_y)^2)
    
    phi = np.arctan((w_x - robot_x) / (w_y - robot_y))

    particles = turnClockwise(particles, -phi-robot_theta)
    particles = forward(particles, distance)

    return particles
