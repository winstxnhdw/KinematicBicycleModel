#!/usr/bin/env python

import numpy as np
from libs.normalise_angle import normalise_angle

class KinematicBicycleModel():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, throttle=0.0, delta=0.0, L=1.0, max_steer=0.7, dt=0.05, c_r=0.0, c_a=0.0):
        """
        2D Kinematic Bicycle Model

        :param x:           (float) vehicle's x-coordinate [m]
        :param y:           (float) vehicle's y-coordinate [m]
        :param yaw:         (float) vehicle's heading [rad]
        :param v:           (float) vehicle's velocity in the x-axis [m/s]
        :param throttle:    (float) vehicle's accleration [m/s^2]
        :param delta:       (float) vehicle's steering angle [rad]
        :param L:           (float) vehicle's wheelbase [m]
        :param max_steer:   (float) vehicle's steering limits [rad]
        :param c_r:         (float) vehicle's aerodynamic coefficient
        :param c_a:         (float) vehicle's coefficient of resistance
        :param dt:          (float) discrete time period [s]

        :return x:          (float) vehicle's x-coordinate [m]
        :return y:          (float) vehicle's y-coordinate [m]
        :return yaw:        (float) vehicle's heading [rad]
        :return v:          (float) vehicle's velocity in the x-axis [m/s]
        :return delta:      (float) vehicle's steering angle [rad]
        :return omega:      (float) vehicle's angular velocity [rad/s]
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
        
        return self.x, self.y, self.yaw, self.v, self.delta, omega

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()
