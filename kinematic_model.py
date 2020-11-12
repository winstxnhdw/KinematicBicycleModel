#!/usr/bin/env python

import numpy as np
from normalise_angle import normalise_angle

class KinematicBicycleModel():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, omega=0.0):

        # Class variables to use whenever within the class when necessary
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega

        self.x_dot = 0.0
        self.y_dot = 0.0

        self.throttle = 0.0
        self.delta = 0.0

        self.dt = 0.1
        
        self.L = 0.0
        self.Lf = 0.0
        self.Lr = self.L - self.Lf

        self.c_r = 0.0
        self.c_a = 0.0

    def kinematic_model(self):

        print("Computing with the kinematic bicycle model")

        # Compute the local velocity in the x-axis
        f_load = self.vx * (self.c_r + self.c_a * self.vx)
        self.vx += self.dt * (self.throttle - f_load)

        # Compute radius and angular velocity of the kinematic bicycle model
        self.delta = np.clip(self.delta, -self.max_steer, self.max_steer)

        if self.delta == 0.0:
            self.omega = 0.0

        else:
            R = self.L / np.tan(self.delta)
            self.omega = self.vx / R

        # Compute the state change rate
        self.x_dot = self.vx * np.cos(self.yaw)
        self.y_dot = self.vx * np.sin(self.yaw)

        # Compute the final state using the discrete time model
        self.x += self.x_dot * self.dt
        self.y += self.y_dot * self.dt
        self.yaw += self.omega * self.dt
        self.yaw = normalise_angle(self.yaw)
        
        return self.x, self.y, self.yaw

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__=="__main__":
    main()
