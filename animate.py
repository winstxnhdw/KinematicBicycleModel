import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import random as rand

from kinematic_model import KinematicBicycleModel
from matplotlib.animation import FuncAnimation
from libs.cubic_spline_pp import generate_cubic_path
from libs.stanley_controller import PathTracker
from libs.car_description import Description

class Simulation:

    def __init__(self):

        fps = 60.0

        self.dt = 1/fps
        self.map_size = 50
        self.frames = 6000
        self.loop = False

class Path:

    def __init__(self):

        # Get path to waypoints.csv
        dir_path = 'data/waypoints.csv'
        df = pd.read_csv(dir_path)

        x = df['X-axis'].values.tolist()
        y = df['Y-axis'].values.tolist()
        ds = 0.05

        self.px, self.py, self.pyaw, _ = generate_cubic_path(x, y, ds)

class Car:

    def __init__(self, sim_params, path_params):

        # Model parameters
        self.x = 40
        self.y = 0
        self.yaw = 0.0
        self.v = 0.0
        self.throttle = 100
        self.delta = 0.0
        self.omega = 0.0
        self.L = 2.5
        self.max_steer = np.deg2rad(33)
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 10.0
        self.ksoft = 1.0
        self.xtrackerr = None
        self.target_id = None

        # Description parameters
        self.length = 4.5
        self.width = 2.0
        self.rear2wheel = 1.0
        self.wheel_dia = 0.15 * 2
        self.wheel_width = 0.2
        self.tread = 0.7

    def drive(self):
        
        self.throttle = rand.uniform(80, 200)
        self.tracker = PathTracker(self.k, self.ksoft, self.max_steer, self.L, self.throttle, self.x, self.y, self.yaw, self.px, self.py, self.pyaw)
        self.throttle, self.delta, xtrackerr, target_id = self.tracker.stanley_control()
        self.kbm = KinematicBicycleModel(self.x, self.y, self.yaw, self.v, self.throttle, self.delta, self.L, self.max_steer, self.dt, self.c_r, self.c_a)
        self.x, self.y, self.yaw, self.v, self.delta, self.omega = self.kbm.kinematic_model()

        self.target_id = target_id
        os.system('cls')
        print("Cross-track error: {}".format(xtrackerr))

def main():
    
    sim = Simulation()
    path = Path()
    car = Car(sim, path)
    desc = Description(car.length, car.width, car.rear2wheel, car.wheel_dia, car.wheel_width, car.tread, car.L)

    interval = sim.dt * 10**3

    fig = plt.figure()
    ax = plt.axes()
    ax.set_box_aspect(1)

    road = plt.Circle((0, 0), 50, color='gray', fill=False, linewidth=30)
    ax.add_patch(road)
    ax.plot(path.px, path.py, '--', color='gold')

    annotation = ax.annotate('{}, {}'.format(car.x, car.y), xy=(car.x, car.y + 5), color='black', annotation_clip=False)
    target, = ax.plot([], [], '+r')

    outline, = ax.plot([], [], color='black')
    fr, = ax.plot([], [], color='black')
    rr, = ax.plot([], [], color='black')
    fl, = ax.plot([], [], color='black')
    rl, = ax.plot([], [], color='black')
    rear_axle, = ax.plot(car.x, car.y, '+', color='black', markersize=2)

    plt.grid()
    
    def animate(frame):

        # Camera tracks car
        ax.set_xlim(car.x - sim.map_size, car.x + sim.map_size)
        ax.set_ylim(car.y - sim.map_size, car.y + sim.map_size)

        # Drive and draw car
        car.drive()
        outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = desc.plot_car(car.x, car.y, car.yaw, car.delta)
        outline.set_data(outline_plot[0], outline_plot[1])
        fr.set_data(fr_plot[0], fr_plot[1])
        rr.set_data(rr_plot[0], rr_plot[1])
        fl.set_data(fl_plot[0], fl_plot[1])
        rl.set_data(rl_plot[0], rl_plot[1])
        rear_axle.set_data(car.x, car.y)

        # Show car's target
        target.set_data(path.px[car.target_id], path.py[car.target_id])

        # Annotate car's coordinate above car
        annotation.set_text('{}, {}'.format(np.around(car.x, 1), np.around(car.y, 1)))
        annotation.set_position((car.x, car.y + 5))

        plt.title('{}s'.format(np.around(sim.dt * frame, 2)), loc='right')
        plt.xlabel('Speed: {} m/s'.format(np.around(car.v, 2)), loc='left')

        return outline, fr, rr, fl, rl, target, rear_axle,

    _ = FuncAnimation(fig, animate, frames=sim.frames, interval=interval, repeat=sim.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=60)
    plt.show()

if __name__ == '__main__':
    main()