import numpy as np
import time

# square constants
SQUARE_X_OFFSET = 200 # adjust these two to position the square
SQUARE_Y_OFFSET = 100
SQUARE_DRAW_SIZE = 500 # adjust this to scale the square
SQUARE_REAL_SIZE = 40 # in cm

# particle constants
NUM_PARTICLES = 100
ROBOT_START_POS = [SQUARE_X_OFFSET, SQUARE_DRAW_SIZE + SQUARE_Y_OFFSET, 0]

# distribution constants
E_MEAN, E_VAR = 0, 1 # in cm
F_MEAN, F_VAR = 0, 1 # in degrees
G_MEAN, G_VAR = 0, 1 # in degrees


def apply_forward(particle, distance):
    """
    particle: the particle to to move forward
    distance: the distance (in cm) that the robot moves forward
    """
    draw_distance = (distance / SQUARE_REAL_SIZE) * SQUARE_DRAW_SIZE
    x, y, theta = particle
    angle = np.deg2rad(theta)
    
    x_rand = np.random.normal(E_MEAN, E_VAR) / SQUARE_REAL_SIZE
    y_rand = np.random.normal(E_MEAN, E_VAR) / SQUARE_REAL_SIZE
    theta_rand = np.random.normal(F_MEAN, F_VAR)
    
    x_new = x + (draw_distance + x_rand) * np.cos(angle)
    y_new = y + (draw_distance + y_rand) * np.sin(angle)
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
    

corners = [(SQUARE_X_OFFSET, SQUARE_Y_OFFSET), 
           (SQUARE_DRAW_SIZE + SQUARE_X_OFFSET, SQUARE_Y_OFFSET), 
           (SQUARE_DRAW_SIZE + SQUARE_X_OFFSET, SQUARE_DRAW_SIZE + SQUARE_Y_OFFSET), (SQUARE_X_OFFSET, SQUARE_DRAW_SIZE + SQUARE_Y_OFFSET)]
lines = []

particles = np.array([ROBOT_START_POS] * NUM_PARTICLES)

for i in range(1, len(corners) + 1):
    lines.append(corners[i - 1] + corners[i % len(corners)])

for line in lines:
    print("drawLine:", str(line))
    
time.sleep(1)
 
for _ in range(4):
    for _ in range(4):
        print("drawParticles:", list(map(tuple, particles)))
        time.sleep(1)
        particles = np.apply_along_axis(lambda p: apply_forward(p, 10), axis=1, arr=particles)
    particles = np.apply_along_axis(lambda p: apply_turn(p, -90), axis=1, arr=particles)

# render for last position
print("drawParticles:", list(map(tuple, particles)))