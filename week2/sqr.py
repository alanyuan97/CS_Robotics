#!/usr/bin/env python
#

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

M_RIGHT = BP.PORT_B
M_LEFT = BP.PORT_C
dps = 300
TURN_VALUE = 1080 # TODO set this value 
TURN_VALUE2 = 210 # TODO set this value
try:
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C
    except IOError as error:
        print(error)

    for index in range(0,4):
    
        BP.set_motor_dps(M_RIGHT,dps+3)
        BP.set_motor_dps(M_LEFT,dps)

        while (BP.get_motor_encoder(M_LEFT)<TURN_VALUE):
            pass
	print(f"Turning with encoder value: {BP.get_motor_encoder(M_RIGHT)}")
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C
        # Turn function
        BP.set_motor_dps(M_LEFT,-50)
        BP.set_motor_dps(M_RIGHT,50)
        
        while (BP.get_motor_encoder(M_RIGHT)<TURN_VALUE2):
            pass
	print(f"Turn end")
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder B
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder C

    BP.reset_all() # Terminate
    try:
       # print(f"Motor B target encoder:{BP.get_motor_encoder(M_RIGHT)}")
       # print(f"Motor C target encoder:{BP.get_motor_encoder(M_LEFT)}")
    except IOError as error:
        print(error)

    #time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

