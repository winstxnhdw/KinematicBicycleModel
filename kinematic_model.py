#!/usr/bin/env python

import numpy as np
from normalise_angle import normalise_angle

class KinematicBicycleModel():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, throttle=0.0, delta=0.0, L=1.0, max_steer=0.7, dt=0.05, c_r=0.0, c_a=0.0):
        """
        Kinematic bicycle model

        :param x:           vehicle's x-coordinate
        :param y:           vehicle's y-coordinate
        :param yaw:         vehicle's heading [rad]
        :param v:          vehicle's velocity in the x-axis [m/s]
        :param throttle:    vehicle's forward speed [m/s]
        :param delta:       vehicle's steering angle [rad]
        :param L:           vehicle's wheelbase [m]
        :param max_steer:   vehicle's steering limits [rad]
        :param c_r:         vehicle's aerodynamic coefficient
        :param c_a:         vehicle's coefficient of resistance
        :param dt:          discrete time period [s]
        """

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.throttle = throttle
        self.delta = delta

        self.dt = dt
        
        self.L = L
        self.max_steer = max_steer

        self.c_r = c_r
        self.c_a = c_a

    def kinematic_model(self):

        # Compute the local velocity in the x-axis
        f_load = self.v * (self.c_r + self.c_a * self.v)
        self.v += self.dt * (self.throttle - f_load)

        # Compute radius and angular velocity of the kinematic bicycle model
        self.delta = np.clip(self.delta, -self.max_steer, self.max_steer)

        if self.delta == 0.0:
            omega = 0.0

        else:
            R = self.L / np.tan(self.delta)
            omega = self.v / R

        # Compute the state change rate
        x_dot = self.v * np.cos(self.yaw)
        y_dot = self.v * np.sin(self.yaw)

        # Compute the final state using the discrete time model
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.yaw += omega * self.dt
        self.yaw = normalise_angle(self.yaw)
        
        return self.x, self.y, self.yaw, self.v, self.delta, self.omega

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__=="__main__":
    main()
