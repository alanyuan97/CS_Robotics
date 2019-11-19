#! /usr/bin/python3
"""
usage:
    `` empty input for update reading
    ` ` input with space for stop motor
    `{dps}` for set turntable dps
    `r` for reset turntable encoder
    `>{pos}` for set encoder of turntable
"""
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division

import sys
import re
import time

import brickpi3 # import the BrickPi3 drivers

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

try:
    try:
        BP.offset_motor_encoder(M_TOP, BP.get_motor_encoder(M_TOP))
    except IOError as error:
        print(error)

    while True:
        line = input()
        if line == "":
            pass

        elif line == " ":
            BP.set_motor_power(M_TOP, 0)

        elif line in ["r", "reset"]:
            BP.offset_motor_encoder(M_TOP, BP.get_motor_encoder(M_TOP))

        elif line.find(">")>=0:
            s = line.find(">")
            BP.set_motor_position(M_TOP, line[s+1:])
        else:
            BP.set_motor_dps(M_TOP, line)

        print(f"POS: {BP.get_motor_encoder(M_TOP)} READ:{BP.get_sensor(S_SONIC)}")

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
