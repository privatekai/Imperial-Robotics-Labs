import time
from math import floor, sqrt, atan, degrees
import brickpi3
from motion import Motion
from particleDataStructures import WALLS, WAYPOINTS, Canvas, Map, Particles
INTERVAL = 20  # cm (= 200 mm steps)
WAYPOINT_TOLERANCE = 1

def navigate_to_waypoint(waypoint, particles, motion):
    # mean of the particles X, Y and theta
    # find angle and distance to get to waypoint
    # move robot (this will update the particles).
    robot_x, robot_y, robot_facing = particles.robot_position()
    w_x, w_y = waypoint

    while sqrt((robot_x - w_x)**2 + (robot_y - w_y)**2) > WAYPOINT_TOLERANCE:
        print("robot_x: ", robot_x)
        print("robot_y: ", robot_y)
        print("robot_facing: ", robot_facing)

        print("w_x: ", w_x)
        print("w_y: ", w_y)

        distance = sqrt((w_x - robot_x)**2 + (w_y - robot_y)**2)
        print("distance: ", distance)

        print("facing_target_rad:", atan((w_y - robot_y)/(w_x - robot_x)))
        facing_target = degrees(atan((w_y - robot_y) / (w_x - robot_x)))
        if (w_x - robot_x) < 0:
            facing_target += 180
        print("facing_target: ", facing_target)
        print("turn: ", facing_target-robot_facing)

        total_target = facing_target - robot_facing
        if total_target > 180:
            total_target -= 360
        elif total_target < -180:
            total_target += 360

        motion.turnAntiClockwise(total_target, particles)

        if distance <= INTERVAL:
            motion.forward(distance * 10, particles)  # cm → mm
            break
        else:
            motion.forward(INTERVAL * 10, particles)  # cm → mm
        # update robot position
        robot_x, robot_y, robot_facing = particles.robot_position()

    # Move the remainder
    # if remainder > 0:
    #     forward(particles, remainder)

def drive_around_map(particles, motion):
    for waypoint in WAYPOINTS:
        navigate_to_waypoint(waypoint, particles, motion)

if __name__ == "__main__":
    BP = brickpi3.BrickPi3()
    try:
        motion = Motion(BP)

        canvas = Canvas()	# global canvas we are going to draw on

        mymap = Map(canvas)
        for wall in WALLS:
            mymap.add_wall(wall)
        mymap.draw()

        particles = Particles(canvas)

        time.sleep(1)

        # print("Enter an x coordinate: ")
        # x_coord = int(input())
        # print("Enter a y coordinate: ")
        # y_coord = int(input())

        # while x_coord != -1:
        #     navigate_to_waypoint((x_coord, y_coord), particles)

        #     print("Enter an x coordinate: ")
        #     x_coord = int(input())
        #     print("Enter a y coordinate: ")
        #     y_coord = int(input())

        drive_around_map(particles, motion)

    finally: # at the end of everything, even with exception.
        BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
