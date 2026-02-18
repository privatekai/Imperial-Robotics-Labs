import time
from math import floor, sqrt, atan, degrees
from motion import BP, LEFT_MOTOR_PORT, MOVEMENT_SPEED, RIGHT_MOTOR_PORT, forward, turnAntiClockwise
from particleDataStructures import WALLS, WAYPOINTS, Canvas, Map, Particles
INTERVAL = 100
WAYPOINT_TOLERANCE = 4

def navigate_to_waypoint(waypoint, particles):
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

        turnAntiClockwise(particles, total_target)
        # full_steps, remainder = divmod(distance, INTERVAL)

        # Move in intervals
        # for _ in range(floor(full_steps)):
        forward(particles, min(100, distance))
        # update robot position
        robot_x, robot_y, robot_facing = particles.robot_position()

    # Move the remainder
    # if remainder > 0:
    #     forward(particles, remainder)

def drive_around_map(particles):
    for waypoint in WAYPOINTS:
        navigate_to_waypoint(waypoint, particles)

if __name__ == "__main__":
    try:
        try:
            BP.offset_motor_encoder(LEFT_MOTOR_PORT, BP.get_motor_encoder(LEFT_MOTOR_PORT)) # reset encoder A
            BP.offset_motor_encoder(RIGHT_MOTOR_PORT, BP.get_motor_encoder(RIGHT_MOTOR_PORT)) # reset encoder D
        except IOError as error:
            print(error)
        
        # Initial motor limits (will be updated in forward() and turnClockwise())
        BP.set_motor_limits(LEFT_MOTOR_PORT, 50, MOVEMENT_SPEED)
        BP.set_motor_limits(RIGHT_MOTOR_PORT, 50, MOVEMENT_SPEED)

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

        drive_around_map(particles)

    finally: # at the end of everything, even with exception.
        BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.