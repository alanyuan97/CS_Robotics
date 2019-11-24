#! /usr/bin/python3

from brickpi3 import BrickPi3

import numpy as np
from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi, radians, degrees

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


# to waypoint
robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C,BrickPi3.PORT_D, BrickPi3.PORT_4, mymap, p_start=(84.0,30.0,0), debug_canvas=canvas)

canvas.drawParticles(robot.p_tuples, robot.p_weights)

##################################
#   FIND AREA A FROM STARTPOINT  #
##################################

# scan bottle
cfg_a_start = radians(-180)
cfg_a_step = radians(1)
cfg_a_end = radians(180)

_x, _y, _t = (84,30,0)

## DEBUG
x_degree = []
y_wall = []
y_bottle = []
y_dis = []
y_expected_dis = []
plt.axis([0,255,-180,180])
plt.xticks(np.arange(-180, 180+1, 60))
fig = plt.figure()
ax = plt.subplot(121)
ax.minorticks_on()
ax.grid(which = 'both',color='blue', alpha = 0.5,linestyle=':',linewidth=0.5)



for r in np.arange(cfg_a_start, cfg_a_end, cfg_a_step):
    _r, _z = robot.get_sonar_dis(r)
    _r += _t

    if _z >= 255:
        continue

    likeli_wall, likeli_bottle = mymap.calculate_likelihood(_x,_y,_r,_z, bottle_detection=True, debug_m=y_expected_dis)

    x_degree.append(degrees(_r))
    y_wall.append(likeli_wall)
    y_bottle.append(likeli_bottle)
    y_dis.append(_z)

robot.set_sonar_rad(0)
sleep(1)

ax.scatter(x_degree,y_wall,alpha=0.5, marker='x', c='orange', s=10)
# ax.scatter(x_degree,y_bottle,alpha=0.5, marker='o',c='blue', s=10)

ax = plt.subplot(122)
ax.minorticks_on()
ax.grid(which = 'both',color='blue', alpha = 0.5,linestyle=':',linewidth=0.5)
ax.scatter(x_degree,y_dis,alpha=0.5, marker='x', c='orange', s=10)
ax.scatter(x_degree,y_expected_dis,alpha=0.5, marker='o', c='red', s=10)


robot.set_sonar_rad(0)
fig.savefig('temp.png')
pass
