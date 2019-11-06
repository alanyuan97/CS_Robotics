#! /usr/bin/python3

import brickpi3
import numpy as np
from math import cos, sin, atan2, pi, sqrt

"""
Direction defintion
1. Let postive x-axi point to right
2. Let postive y-axi point to up
3. Let angle theta be the angle arctan(y/x)

Unit of measurement
1. Unit of distance is cm
2. Unit of angle is radian
3. Prameter proportional to angle or distance need to be normalised
"""

class RobotBase(brickpi3.BrickPi3):
    def __init__(self, M_LEFT, M_RIGHT,*, p_count=100, gaussian_e=(0,0.003), gaussian_f=(0, 0.00001/180*pi), gaussian_g=(0,0.001/180*pi)):
        # BP init
        super(RobotBase, self).__init__()
        self.M_LEFT = M_LEFT
        self.M_RIGHT = M_RIGHT

        # base characteristics config
        self.stright_dps = 300
        self.stright_dpcm = 27.0 # degree of motor rotation to move 1 cm
        self.turn_dps = 50
        self.turn_dpradian = 4440.0/5/2/pi # degree of motor rotation to trun 1 radian


        # localisation init
        # NOTE: p_weights and p_tuples need have same order
        self.p_count = p_count
        self.p_weights = [1.0/p_count] * p_count
        self.p_tuples = [(0.0, 0.0, 0.0)] * p_count # (x,y,theta)

        # config for gaussian noise, all normalised by cm, or by radian
        self.gaussian_e = gaussian_e
        self.gaussian_f = gaussian_f
        self.gaussian_g = gaussian_g

    def get_est_pos(self):
        return np.average(self.p_tuples, axis=0, weights=self.p_weights)

    def to_waypoint(self, x, y):
        # get estimation of current location
        est_x, est_y, est_t = self.get_est_pos()
        est_t = normalise_anlge(est_t)

        # calculate movement, abort it needed
        diff_x = x - est_x
        diff_y = y - est_y
        target_t = atan2(diff_y, diff_x)

        relative_t = target_t - est_t
        relative_t = normalise_anlge(relative_t) # bond turning angle to +- pi
        relative_d = sqrt(diff_x*diff_x + diff_y*diff_y)

        print(f"turn {relative_t}, go {relative_d}")
        if relative_d < 0.001:
            print("Warning: distance to move too small, ignore command")
            return

        # perform movement
        self.to_relative_turn(relative_t)
        self.to_relative_forward(relative_d)

        # TODO: recalibration based on sensor

    def to_relative_forward(self, distance):
        # perform movement
        self.offset_motor_encoder(self.M_RIGHT, self.get_motor_encoder(self.M_RIGHT))
        self.offset_motor_encoder(self.M_LEFT, self.get_motor_encoder(self.M_LEFT))
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, self.stright_dps )
        while (self.get_motor_encoder(self.M_RIGHT) < self.stright_dpcm*distance):
            pass
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, 0)

        # update model
        inc_distance = np.random.normal(*np.array(self.gaussian_e)*abs(distance), self.p_count) + distance
        err_f = np.random.normal(*np.array(self.gaussian_f)*distance, self.p_count)
        for i in range(len(self.p_tuples)):
            _x, _y, _t = self.p_tuples[i]
            self.p_tuples[i] = (_x + cos(_t)*inc_distance[i],
                                _y + sin(_t)*inc_distance[i],
                                _t + err_f[i])


    def to_relative_turn(self, angle):
        # perform movement
        self.offset_motor_encoder(self.M_RIGHT, self.get_motor_encoder(self.M_RIGHT)) # reset encoder B
        self.offset_motor_encoder(self.M_LEFT, self.get_motor_encoder(self.M_LEFT)) # reset encoder C
        # Turn function
        if angle>0:
            self.set_motor_dps(self.M_LEFT, -self.turn_dps)
            self.set_motor_dps(self.M_RIGHT, self.turn_dps)
            while (self.get_motor_encoder(self.M_RIGHT) < angle*self.turn_dpradian):
                pass
        else:
            self.set_motor_dps(self.M_LEFT, self.turn_dps)
            self.set_motor_dps(self.M_RIGHT, -self.turn_dps)
            while (self.get_motor_encoder(self.M_RIGHT) > angle*self.turn_dpradian):
                pass
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, 0)

        # update model
        err_g = np.random.normal(*np.array(self.gaussian_g)*abs(angle), self.p_count)
        for i in range(len(self.p_tuples)):
            _x, _y, _t = self.p_tuples[i]
            self.p_tuples[i] = (_x ,
                                _y ,
                                _t + angle+ err_g[i])

def normalise_anlge(theta):
    t = theta % (2*pi) #modulus will have the same sign as the denominator
    if t > pi:
        t -= 2*pi
    return t





