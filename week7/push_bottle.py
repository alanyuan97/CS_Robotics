#! /usr/bin/python3

from brickpi3 import BrickPi3

from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi

canvas = Canvas(210)
mymap = Map()

# to waypoint
robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C,BrickPi3.PORT_D, BrickPi3.PORT_4, mymap, p_start=(84.0,30.0,0), debug_canvas=canvas)

for _ in range(3):
    robot.touch_bottle(200)
    sleep(1)
