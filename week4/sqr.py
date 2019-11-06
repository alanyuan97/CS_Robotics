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

def s_print():
    lines = [(100,100),(100,700),(700,700),(700,100)]
    for i in range(0,3):
        line = (*lines[i],*lines[i+1])
        print("drawLine:"+ str(line))
    line = (*lines[3],*lines[0])
    print("drawLine:"+ str(line))
    return

# init
try:
    s_print()

    for _ in range(4):
        for _ in range(4):
            robot.to_relative_forward(10) # *task unpack to arguments
            myprint(robot.p_tuples)
            print (f"est postion:{robot.get_est_pos()}")
            sleep(1)

        robot.to_relative_turn(0.5*pi)
        myprint(robot.p_tuples)
        sleep(1)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    robot.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

