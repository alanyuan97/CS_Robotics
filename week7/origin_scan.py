#! /usr/bin/python3

from brickpi3 import BrickPi3

import numpy as np
from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi, radians, degrees,atan2
import matplotlib.pyplot as plt

canvas = Canvas(210)

mymap = Map()


# draw wall

# Definitions of walls

# a: O to A

# b: A to B

# c: C to D

# d: D to E

# e: E to F

# f: F to G

# g: G to H

# h: H to O

mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
canvas.drawLines(mymap.walls)

name_list = ["a","b","c","d","e","f","g","h"]

robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C,BrickPi3.PORT_D, BrickPi3.PORT_4, mymap, p_start=(84.0,30.0,0), debug_canvas=canvas)

canvas.drawParticles(robot.p_tuples, robot.p_weights)

cfg_scan_a_range = (radians(-30), radians(30), radians(2))
cfg_scan_b_range = (radians(-30), radians(-200), radians(-2))
cfg_scan_a_var = 0.4
cfg_scan_b_var = 0.4

#####################

#       AREA C      #

#####################

# navigate to Area B

robot.to_waypoint(42,63)
robot.set_motor_position(robot.M_SONAR, 130)

original_theta=atan2(42,33)
robot.to_relative_turn(-original_theta)
robot.sonar_calibrate(pi/2)
robot.sonar_calibrate(pi)

# a_bottle, a_walls = robot.identify_bottle(a_obs)
# _x,_y = robot.update_pos(a_walls)
# print(f"update: X{_x}, Y: {_y}")
robot.touch_bottle(500)

# go to AREA B
robot.to_relative_backward(10)

robot.sonar_calibrate(pi)
robot.sonar_calibrate(pi/2)

robot.to_waypoint(126,53)
robot.sonar_calibrate(-pi/2)
robot.to_waypoint(126,100)
robot.sonar_calibrate(-pi/2)

_,_,_t = robot.get_pos_mean()
robot.to_relative_turn(_t - pi/2)

#####################

#       AREA B      #

#####################

# navigate to Area B

robot.touch_bottle(500)


# go to AREA A
robot.to_relative_turn(radians(180))
robot.to_waypoint(126,42)

robot.sonar_calibrate(0)
robot.to_relative_turn(radians(90))


#####################

#       AREA A     #

#####################

# navigate to origin

robot.touch_bottle(500)
robot.sonar_calibrate(-pi/2)
# go to checkpoint 2
# robot.to_relative_turn(atan2(12,84))
# robot.to_relative_backward(85) # sqrt( 12**2 + 84**2 ) .= 85
o_obs = robot.get_nearest_obstacles(*cfg_scan_b_range,cfg_scan_b_var,True)
o_bottle, o_walls = robot.identify_bottle(o_obs)
_x,_y = robot.update_pos(o_walls)
print(f"Origin update: X{_x}, Y: {_y}")
robot.to_waypoint(84,30)
