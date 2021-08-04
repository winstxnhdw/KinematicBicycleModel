#!/usr/bin/env python

import numpy as np

from libs.normalise_angle import normalise_angle

class KinematicBicycleModel():

    def __init__(self, L=1.0, max_steer=0.7, dt=0.05, c_r=0.0, c_a=0.0):
        """
        2D Kinematic Bicycle Model

        At initialisation
        :param L:           (float) vehicle's wheelbase [m]
        :param max_steer:   (float) vehicle's steering limits [rad]
        :param dt:          (float) discrete time period [s]
        :param c_r:         (float) vehicle's aerodynamic coefficient
        :param c_a:         (float) vehicle's coefficient of resistance

        At every time step
        :param x:           (float) vehicle's x-coordinate [m]
        :param y:           (float) vehicle's y-coordinate [m]
        :param yaw:         (float) vehicle's heading [rad]
        :param v:           (float) vehicle's velocity in the x-axis [m/s]
        :param throttle:    (float) vehicle's accleration [m/s^2]
        :param delta:       (float) vehicle's steering angle [rad]

        :return x:          (float) vehicle's x-coordinate [m]
        :return y:          (float) vehicle's y-coordinate [m]
        :return yaw:        (float) vehicle's heading [rad]
        :return v:          (float) vehicle's velocity in the x-axis [m/s]
        :return delta:      (float) vehicle's steering angle [rad]
        :return omega:      (float) vehicle's angular velocity [rad/s]
        """

        self.dt = dt
        self.L = L
        self.max_steer = max_steer
        self.c_r = c_r
        self.c_a = c_a

    def kinematic_model(self, x, y, yaw, v, throttle, delta):

        # Compute the local velocity in the x-axis
        f_load = v * (self.c_r + self.c_a * v)
        v += self.dt * (throttle - f_load)

        # Compute radius and angular velocity of the kinematic bicycle model
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        # Compute the state change rate
        x_dot = v * np.cos(yaw)
        y_dot = v * np.sin(yaw)
        omega = v * np.tan(delta) / self.L

        # Compute the final state using the discrete time model
        x += x_dot * self.dt
        y += y_dot * self.dt
        yaw += omega * self.dt
        yaw = normalise_angle(yaw)
        
        return x, y, yaw, v, delta, omega

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == "__main__":
    main()
