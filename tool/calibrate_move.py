#! /usr/bin/python3
"""
usage:
    `` empty input for update reading
    ` ` input with space for stop motor
    `{precent} {precent}` for set motor power
    `r` for reset reading
"""
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division

import sys
import re

import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

M_RIGHT = BP.PORT_B
M_LEFT = BP.PORT_C

try:
    try:
        BP.offset_motor_encoder(M_RIGHT, BP.get_motor_encoder(M_RIGHT))
        BP.offset_motor_encoder(M_LEFT, BP.get_motor_encoder(M_LEFT))
    except IOError as error:
        print(error)

    while True:
        line = input()
        if line == "":
            pass

        elif line == " ":
            BP.set_motor_power(M_RIGHT | M_LEFT, 0)

        elif line in ["r", "reset"]:
            BP.offset_motor_encoder(M_RIGHT, BP.get_motor_encoder(M_RIGHT))
            BP.offset_motor_encoder(M_LEFT, BP.get_motor_encoder(M_LEFT))

        elif line.find(" ") > 0:
            s = line.find(" ")
            BP.set_motor_dps(M_RIGHT, line[:s])
            BP.set_motor_dps(M_LEFT, line[s+1:])

        print(f"R: {BP.get_motor_encoder(M_LEFT)} L:{BP.get_motor_encoder(M_RIGHT)}")

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
