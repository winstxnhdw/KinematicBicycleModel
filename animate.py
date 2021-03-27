import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

from kinematic_model import KinematicBicycleModel
from matplotlib.animation import FuncAnimation
from libs.cubic_spline_planner import calc_spline_course
from libs.stanley_controller import PathTracker

class Simulation:

    def __init__(self):

        fps = 60.0

        self.dt = 1/fps
        self.map_size = 50
        self.frames = 10000
        self.loop = False

class Path:

    def __init__(self):

        # Get path to waypoints.csv
        dir_path = 'data/waypoints.csv'
        df = pd.read_csv(dir_path)

        x = df['X-axis'].values.tolist()
        y = df['Y-axis'].values.tolist()
        ds = 0.1

        self.px, self.py, self.pyaw = calc_spline_course(x, y, ds)

class Car:

    def __init__(self, sim_params, path_params):

        self.x = 40
        self.y = 0
        self.yaw = 0.0
        self.v = 0.0
        self.throttle = 100.0
        self.delta = 0.0
        self.omega = 0.0
        self.L = 3
        self.max_steer = np.deg2rad(33)
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0

        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw

        self.k = 10.0
        self.ksoft = 1.0
        self.cg2frontaxle = self.L/2

        self.xtrackerr = None

    def drive(self):
        
        self.tracker = PathTracker(self.k, self.ksoft, self.max_steer, self.cg2frontaxle, self.throttle, self.x, self.y, self.yaw, self.px, self.py, self.pyaw)
        self.throttle, self.delta, xtrackerr = self.tracker.stanley_control()
        self.kbm = KinematicBicycleModel(self.x, self.y, self.yaw, self.v, self.throttle, self.delta, self.L, self.max_steer, self.dt, self.c_r, self.c_a)
        self.x, self.y, self.yaw, self.v, self.delta, self.omega = self.kbm.kinematic_model()
        os.system('cls')
        print("Cross-track error: {}".format(xtrackerr))

def main():
    
    sim = Simulation()
    path = Path()
    car = Car(sim, path)

    interval = sim.dt * 10**(-3)

    fig = plt.figure()
    ax = plt.axes()
    vehicle, = ax.plot(car.x, car.y, 'ro', markersize=15)
    ax.plot(path.px, path.py)
    ax.set_aspect('equal', adjustable='box')

    annotation = ax.annotate('{}, {}'.format(car.x, car.y), xy=(car.x, car.y + 5), annotation_clip=False)

    def animate(i):

        ax.set_xlim(car.x - sim.map_size, car.x + sim.map_size)
        ax.set_ylim(car.y - sim.map_size, car.y + sim.map_size)
        ax.annotate
        car.drive()
        vehicle.set_data(car.x, car.y)
        annotation.set_text('{}, {}'.format(np.around(car.x, 1), np.around(car.y, 1)))
        annotation.set_position((car.x, car.y + 5))
        plt.title('{} m/s'.format(np.around(car.v, 2)), loc='right')
        return vehicle,

    anim = FuncAnimation(fig, animate, frames=sim.frames, interval=interval, repeat=sim.loop)

    plt.grid()
    plt.show()

if __name__ == '__main__':
    main()