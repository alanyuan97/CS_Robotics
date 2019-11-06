#! /usr/bin/python3

from brickpi3 import BrickPi3

from RobotBase import RobotBase
from time import sleep
from math import pi

robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C)

def myprint(data):
    TUPLE = []
    for i in range(len(data)):
        TUPLE.append((data[i][0]*15+100,data[i][1]*15+100,data[i][2])) #convert list of list => list of tuples, with offset to shift graph
    print("drawParticles:"+str(TUPLE))
    return

def axi_print():
    lines = [(700,100), (100,100), (100,700)]
    for i in range(0,2):
        line = (*lines[i],*lines[i+1])
        print("drawLine:"+ str(line))
    return

# init
try:
    axi_print()
    myprint(robot.p_tuples)

    while True:
        line = input()
        s = line.find(",")
        _x = int(line[:s])
        _y = int(line[s+1:])
        robot.to_waypoint(_x, _y)
        print(robot.get_est_pos())
        myprint(robot.p_tuples)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    robot.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

