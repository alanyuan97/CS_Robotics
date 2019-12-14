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



cfg_scan_a_range = (radians(-180), radians(180), radians(2))

cfg_scan_b_range = (radians(-30), radians(-200), radians(-2))

cfg_scan_a_var = 0.4

cfg_scan_b_var = 0.4



robot.to_waypoint(126,83)

# original_theta=atan2(53,42) # 83=30= 53, 126-84=42

# robot.to_relative_turn(pi/2-original_theta)



robot.to_waypoint(126,140) # Face B

# robot.to_waypoint(126,141) # Face B 

robot.set_motor_position(robot.M_SONAR, 130) # Trigger extension arm



robot.sonar_calibrate(-pi/2) # R -> 

robot.sonar_calibrate(pi/2)  # <- R 



robot.touch_bottle(500)

# scan bottle

#####################

#       AREA B      #

#####################

# navigate to Area C



# if(not b_hit):

#     # arrive a_hit without touch bottle

#     b_obstacles = robot.get_nearest_obstacles(*cfg_scan_a_range, cfg_scan_a_var, DEBUG=True)

#     bottles, walls = robot.identify_bottle(b_obstacles)

#     x,y=robot.update_pos(walls)

#     print("x_B: ", x)

#     print("y_B: ", y)

#     #try:

#     b_bottle_rad, b_bottle_dis = min(bottles, key=lambda t:t[1])

#     # except:

#     #     pass

#     robot.to_relative_turn(b_bottle_rad)

#     robot.touch_bottle(300)

#     robot.to_waypoint(126,140)





robot.to_waypoint(126,135) # face down

robot.sonar_calibrate(pi/2)

robot.sonar_calibrate(-pi/2)

a_hit = robot.to_waypoint(126,42)



if not a_hit:

    robot.sonar_calibrate(0)  # Downwards | R



    robot.to_relative_turn(radians(90)) # Face A

    robot.touch_bottle(500)



    robot.sonar_calibrate(-pi/2) # Downwards | R



robot.to_waypoint(42,90)

robot.to_waypoint(42,100) # Face C

robot.sonar_calibrate(pi/2) # <- R



robot.touch_bottle(500)



robot.sonar_calibrate(pi/2) # <- R



o_obs = robot.get_nearest_obstacles(*cfg_scan_a_range,cfg_scan_a_var,True)

o_bottle, o_walls = robot.identify_bottle(o_obs)

_x,_y = robot.update_pos(o_walls)

print(f"Origin update: X{_x}, Y: {_y}")

robot.to_waypoint(84,30)