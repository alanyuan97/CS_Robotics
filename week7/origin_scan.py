#! /usr/bin/python3

from brickpi3 import BrickPi3

from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi,degrees, radians

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
mymap.add_wall((0,0,0,168))        # a
mymap.add_wall((0,168,84,168))     # b
mymap.add_wall((84,126,84,210))    # c
mymap.add_wall((84,210,168,210))   # d
mymap.add_wall((168,210,168,84))   # e
mymap.add_wall((168,84,210,84))    # f
mymap.add_wall((210,84,210,0))     # g
mymap.add_wall((210,0,0,0))        # h
canvas.drawLines(mymap.walls)
name_list = ["a","b","c","d","e","f","g","h"]


# to waypoint
robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C,BrickPi3.PORT_D, BrickPi3.PORT_4, mymap, p_start=(84.0,30.0,0), debug_canvas=canvas)
waypoints = [(180,30), (180,54), (138,54), (138,168), (114,168), (114,84), (84,84), (84,30)]
calpoints = [(100,30),(120,30),(140,30),(160,30)]

# canvas.drawPoints(waypoints)

# canvas.drawParticles(robot.p_tuples, robot.p_weights)
# print(f"location {robot.get_pos_mean()}")
# sleep(1)./
result = robot.get_obstacles(-pi,pi,radians(2),0.4,True)
print(f"Scanned angles : {result}")