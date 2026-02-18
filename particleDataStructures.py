#!/usr/bin/env python 

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import math

import brickpi3 # import the BrickPi3 drivers
from sonarSensor import SonarSensor

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
sonar = SonarSensor(BP)

# Sensor configuration is now handled by the SonarSensor class

# particle constants
NUM_PARTICLES = 100
ROBOT_START_POS = (84, 30, 0, 1/NUM_PARTICLES)

# distribution constants
E_MEAN, E_VAR = 0, 10 # in cm
F_MEAN, F_VAR = 0, 1 # in degrees
G_MEAN, G_VAR = 0, 1 # in degrees
SONAR_VAR = 4 # in cm

# waypoints (in cm)
WAYPOINTS = [
    # (84, 30), Commenting out as this is the robot start pos
    (180, 30),
    (180, 54),
    (138, 54),
    (138, 168),
    (114, 116),
    (114, 84),
    (84, 84),
    (84, 30),
]

# Walls (in cm)
WALLS = [
    (0,0,0,168),        # a: O to A
    (0,168,84,168),     # b: A to B
    (84,126,84,210),    # c: C to D
    (84,210,168,210),   # d: D to E
    (168,210,168,84),   # e: E to F
    (168,84,210,84),    # f: F to G
    (210,84,210,0),     # g: G to H
    (210,0,0,0),        # h: H to O
]

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)) # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)) # in cm

def calcW():
    return random.random()

def calcTheta():
    return random.randint(0,360)

def normPdf(x, variance):
    return math.exp((-x**2) / (2 * variance)) / math.sqrt((2 * math.pi * variance)) 

""" 
Data Structures! 
"""

# A Canvas class for drawing a map and particles:
# 	- it takes care of a proper scaling and coordinate transformation between
#	  the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin      = 0.05*map_size
        self.scale       = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self,line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data]
        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self, canvas):
        self.walls = []
        self.canvas = canvas

    def add_wall(self,wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            self.canvas.drawLine(wall)

# Simple Particles set
class Particles:
    def __init__(self, canvas):
        self.n = NUM_PARTICLES 
        self.data = [ROBOT_START_POS] * NUM_PARTICLES
        self.canvas = canvas

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)]
    
    def draw(self):
        self.canvas.drawParticles(self.data)
    
    def forward(self, distance):
        """
        distance: the distance (in cm) that the robot moves forward
        """
        self.data = self.__apply_forward(distance)
        self.draw()
        self.__MCL_update()
    
    def turn(self, angle):
        """
        angle: the angle (in degrees) that the robot rotates
        """
        self.data = self.__apply_turn(angle)
        self.draw()
        self.__MCL_update()

    def robot_position(self):
        """
        Takes the average of the world coordinate particles to give an estimate of the robot's real coordinates.
        """
        x, y, theta = (0, 0, 0)
        for _x, _y, _theta, _w in self.data:
            x += _x * _w
            y += _y * _w
            theta += _theta * _w

        return (x, y, theta)
        
    def __apply_forward(self, distance):
        """
        distance: the distance (in cm) that the robot moves forward
        """    
        new_data = []
        for x, y, theta, weight in self.data:

            angle = math.radians(theta)
            dist_rand = random.gauss(E_MEAN, E_VAR**0.5)
            theta_rand = random.gauss(F_MEAN, F_VAR**0.5)
            
            x_new = x + (distance + dist_rand) * math.cos(angle)
            y_new = y + (distance + dist_rand) * math.sin(angle)
            theta_new = theta + theta_rand

            new_data.append((x_new, y_new, theta_new, weight))
        return new_data

    def __apply_turn(self, angle):
        """
        angle: the angle (in degrees) that the robot rotates
        """
        new_data = []
        for x, y, theta, weight in self.data:
            theta_rand = random.gauss(G_MEAN, G_VAR**0.5)
            new_data.append((x, y, theta + angle + theta_rand, weight))
            
        return new_data

    def __update_weight(self, measured_distance):
        """
        particle: the particle
        measured_distance: the distance measured from the sonar
        """
        new_data = []
        for x, y, theta, _ in self.data:
            angle = math.radians(theta)
            
            # calculate the particle's distance from each wall and take the closest one.
            minimum_distance_to_wall = float("inf") 
            for wall in WALLS:
                a_x, a_y, b_x, b_y = wall

                # TODO: divide by zero error
                
                distance_to_wall = (b_y - a_y) * (a_x - x) - (b_x - a_x) * (a_y - y)
                distance_to_wall /= (b_y - a_y) * math.cos(angle) - (b_x - a_x) * math.sin(angle)
                
                # check if the intersection is between the endpoints of the wall.
                x_intersection = x + distance_to_wall * math.cos(angle)
                y_intersection = y + distance_to_wall * math.sin(angle)
                min_x, max_x = min(a_x, b_x), max(a_x, b_x)
                min_y, max_y = min(a_y, b_y), max(a_y, b_y)
                if x_intersection < min_x or max_x < x_intersection or y_intersection < min_y or max_y < y_intersection:
                    continue
                
                # change minimum distance if this wall is closer
                # distance must be positive because the wall should be in front of the robot
                if distance_to_wall < 0 or minimum_distance_to_wall < distance_to_wall:
                    continue
                minimum_distance_to_wall = distance_to_wall
            
            delta_distance = measured_distance - minimum_distance_to_wall
            new_weight = normPdf(delta_distance, variance=SONAR_VAR) # we take the root of the variance as scale corresponds to standard deviation
            new_data.append((x, y, theta, new_weight))
            
        return new_data
    
    def __normalise_particles(self):
        total_weight = sum(weight for (_, _, _, weight) in self.data)
        if total_weight == 0:
            raise Excpetion("total_weight is 0 - particles are gonna die")
        return [(x, y, theta, weight / total_weight) for (x, y, theta, weight) in self.data]
    
    def __resample_particles(self):
        cumulative_weight = 0.0
        cumulative_weight_array = []
        for (x, y, theta, weight) in self.data:
            cumulative_weight += weight
            cumulative_weight_array.append((x, y, theta, cumulative_weight))

        sampled_array = []
        for _ in range(100):
            generated_weight = random.uniform(0, 1)
            for (x, y, theta, cumulative_weight) in cumulative_weight_array:
                if generated_weight <= cumulative_weight:
                    sampled_array.append((x, y, theta, 1/NUM_PARTICLES))
                    break
        
        return sampled_array
    
    def __MCL_update(self):
        measured_distance = sonar.get_distance()
        if measured_distance is not None:
            print(measured_distance)  # print the calibrated distance in CM
        
        time.sleep(0.05)
        self.data = self.__update_weight(measured_distance)
        self.draw()
        time.sleep(0.05)
        self.data = self.__normalise_particles()
        self.data = self.__resample_particles()
        self.draw()
        time.sleep(0.05)
            

if __name__ == "__main__":
    canvas = Canvas()	# global canvas we are going to draw on

    mymap = Map(canvas)
    for wall in WALLS:
        mymap.add_wall(wall)
    mymap.draw()

    particles = Particles(canvas)

    t = 0
    while True:
        particles.update()
        particles.draw()
        t += 0.05
        time.sleep(0.05)