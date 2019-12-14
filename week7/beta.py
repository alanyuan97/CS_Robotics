#! /usr/bin/python3



from brickpi3 import BrickPi3



import numpy as np

from helper import RobotBase, Canvas, Map

from time import sleep

from math import pi, radians, degrees,atan2

# import matplotlib.pyplot as plt



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

def faceNorth():

    _,_,_t = robot.get_pos_mean()

    robot.to_relative_turn(pi/2 - _t)

def faceSouth():

    _,_,_t = robot.get_pos_mean()

    robot.to_relative_turn(-pi/2 - _t)

def faceEast():

    _,_,_t = robot.get_pos_mean()

    robot.to_relative_turn(-_t)



canvas.drawParticles(robot.p_tuples, robot.p_weights)



cfg_scan_a_range = (radians(-100), radians(100), radians(2))

cfg_scan_b_range = (radians(-30), radians(-200), radians(-2))

cfg_scan_a_var = 0.4

cfg_scan_b_var = 0.4



robot.to_waypoint(126,83)

robot.set_motor_position(robot.M_SONAR, 130) # Trigger extension arm

faceNorth()

#if the y-coordinate of bottle in area B is about 84,then 

#robot.to_relative_backward(2)



# original_theta=atan2(53,42) # 83=30= 53, 126-84=42

# robot.to_relative_turn(pi/2-original_theta)

robot.sonar_calibrate(-pi/2)

robot.touch_bottle()

robot.sonar_calibrate(-pi/2) # R ->



faceSouth()

#robot.to_waypoint(126,135) # face down
# robot.to_relative_turn(radians(180))

robot.sonar_calibrate(pi/2)



faceSouth()
a_hit = robot.to_waypoint(126,60)

if not a_hit:

    robot.sonar_calibrate(0)
    robot.sonar_calibrate(pi/2)

    faceEast() # Face A

    #robot.to_relative_turn(radians(90))

    robot.touch_bottle()
    robot.sonar_calibrate(0)
    robot.sonar_calibrate(-pi/2) # Downwards | R



c_hit = robot.to_waypoint(42,65)

if not c_hit:
    faceNorth()
    robot.sonar_calibrate(pi/2) # <- R
    robot.touch_bottle()
    robot.sonar_calibrate(pi/2) # <- R

robot.to_waypoint(84,42)

faceSouth()

robot.sonar_calibrate(0) # <- R

robot.sonar_calibrate(-pi/2) # Downwards R
robot.to_waypoint(84,30)
