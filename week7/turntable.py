#! /usr/bin/python3

## config
turntable_dpe = - 1260.0/180 # truntable rotation degree per rotation
turntable_step = 1
turn_start = -190
turn_end = -170

turn_start_1 = -105
turn_end_1 = -75

turn_start_angle = -70
turn_end_angle = -50

offset = 3

n1=[]
n2=[]

import sys
import re
import time
import math

import brickpi3 # import the BrickPi3 drivers
import numpy as np
import matplotlib as mt
import matplotlib.pyplot as plt

# def find_obstacles(distance, degree):
#     """
#     Result returns a list of (dis, degree), indicating the center of obstacle
#     """
#     result = []

#     # find list of walls
#     max_diff = 5
#     min_point = 3

#     t_i = None, t_dis = None
#     for i, (dis,degree) in enumerate(zip(result_distance, result_degree)):
#         if t_i is None:
#             t_i = i
#             t_dis = dis

#         else:
#             if abs(t_dis - dis)<max_diff:
#                 continue
#             else:
#                 # cal mid point
#                 if i-t_i > min_point:
#                     result.append(())

#                 t_i, t_dis = i, dis


        


class BrickPi333(brickpi3.BrickPi3):
    def __init__(self):
        super().__init__()
        time.sleep(1)
        self.reset_all()
        time.sleep(1)

    def __del__(self):
        self.reset_all()
        time.sleep(1)

BP = BrickPi333() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.


M_TOP = BP.PORT_D
S_SONIC = BP.PORT_4
BP.set_sensor_type(S_SONIC, BP.SENSOR_TYPE.NXT_ULTRASONIC)
BP.set_motor_position_kp(M_TOP,60)
# origin_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start,turn_end,n1)
# x_initial = np.amin(n1) + offset

# origin_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start_1,turn_end_1,n2)
# y_initial = np.amin(n2) + offset 
BP.set_motor_position_kd(M_TOP,75)


def origin_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start,turn_end,n):

    result_distance = []
    result_degree = []

    try:

        # plt.axis([0,255,-180,180])
        # plt.xticks(np.arange(-180, 180+1, 60))
        # # plt.xlim(-180,180)


        # fig = plt.figure()
        # ax = plt.subplot(111)
        # plt.grid(color='blue',linestyle='-',linewidth=0.5)

        x_initial = 0
        y_initial = 0

        while BP.get_motor_status(M_TOP)[3]!=0:
            pass

        for d in np.arange(turn_start, turn_end, turntable_step):
            BP.set_motor_position(M_TOP, d*turntable_dpe)
            while int(BP.get_motor_encoder(M_TOP)/turntable_dpe)!=d:
                pass
            time.sleep(0.001)
            print(d)

            while True:
                try:
                    dis = BP.get_sensor(S_SONIC)
                    print(dis)
                    n.append(dis)
                    break
                except Exception as e:
                    time.sleep(0.5)
                    print(e)


            if dis==255:
                
                continue

        #BP.set_motor_position(M_TOP,0)
        while BP.get_motor_status(M_TOP)[3]!=0:
            pass
        time.sleep(1)
        np.savetxt('temp.nptxt',(result_degree, result_distance))

    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

def angle_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start,turn_end,y):
    #try:
    BP.set_motor_position(M_TOP, -90*turntable_dpe)

    while True:
        try: 
            dis = BP.get_sensor(S_SONIC)
            break
        except Exception as e:
            time.sleep(0.5)
            print(e)
    #print(dis)
    angle = math.cos(y/dis)
    return angle


origin_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start,turn_end,n1)
x_initial = np.amin(n1) + offset

origin_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start_1,turn_end_1,n2)
y_initial = np.amin(n2) + offset 

angle = angle_calibration(M_TOP, S_SONIC, BP, turntable_dpe,turntable_step,turn_start_angle,turn_end_angle,y_initial)
#angle = (140-t_angle)/2
BP.set_motor_position(M_TOP,0)
print("x: ", x_initial)
print("y: ", y_initial)
print(angle)
time.sleep(1)
