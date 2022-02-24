import numpy as np

from numpy import ndarray
from math import cos, sin

class Description:

    def __init__(self, overall_length: float, overall_width: float, rear_overhang: float, tyre_diameter: float, tyre_width: float, axle_track: float, wheelbase: float):
        
        """
        Description of a car for visualising vehicle control in Matplotlib.
        
        At initialisation
        :param overall_length:      (float) vehicle's overall length [m]
        :param overall_width:       (float) vehicle's overall width [m]
        :param rear_overhang:       (float) distance between the rear bumper and the rear axle [m]
        :param tyre_diameter:       (float) vehicle's tyre diameter [m]
        :param tyre_width:          (float) vehicle's tyre width [m]
        :param axle_track:          (float) vehicle's axle track [m]
        :param wheelbase:           (float) vehicle's wheelbase [m]
        
        At every time step
        :param x:                   (float) x-coordinate of the vehicle's rear axle
        :param y:                   (float) y-coordinate of the vehicle's rear axle
        :param yaw:                 (float) vehicle's heading [rad]
        :param steer:               (float) vehicle's steering angle [rad]
        
        :return outlines:           (ndarray) vehicle's outlines [x, y]
        :return fr_wheel:           (ndarray) vehicle's front-right axle [x, y]
        :return rr_wheel:           (ndarray) vehicle's rear-right axle [x, y]
        :return fl_wheel:           (ndarray) vehicle's front-left axle [x, y]
        :return rl_wheel:           (ndarray) vehicle's rear-right axle [x, y]
        """

        centreline_to_wheel = axle_track / 2
        centreline_to_side = overall_width / 2
        rear_axle_to_front_bumper = overall_length - rear_overhang

        vehicle_vertices = np.array([(-rear_overhang,              centreline_to_side),
                                     ( rear_axle_to_front_bumper,  centreline_to_side),
                                     ( rear_axle_to_front_bumper, -centreline_to_side),
                                     (-rear_overhang,             -centreline_to_side)])

        wheel_vertices = np.array([(-tyre_diameter,   tyre_width  - centreline_to_wheel),
                                   ( tyre_diameter,   tyre_width  - centreline_to_wheel),
                                   ( tyre_diameter, (-tyre_width) - centreline_to_wheel),
                                   (-tyre_diameter, (-tyre_width) - centreline_to_wheel)])

        self.outlines = np.concatenate((vehicle_vertices, [vehicle_vertices[0]]))

        self.wheel_format = np.concatenate((wheel_vertices, [wheel_vertices[0]]))
        self.rl_wheel = self.wheel_format.copy()
        self.rl_wheel[:, 1] *= -1
        self.fl_wheel = self.rl_wheel.copy()
        self.fl_wheel[:, 0] += wheelbase 
        self.fr_wheel = self.wheel_format.copy()
        self.fr_wheel[:, 0] += wheelbase
                                   
        self.fr_wheel_centre = np.array([(self.fr_wheel[0][0] + self.fr_wheel[2][0]) / 2,
                                         (self.fr_wheel[0][1] + self.fr_wheel[2][1]) / 2])

        self.fl_wheel_centre = np.array([(self.fl_wheel[0][0] + self.fl_wheel[2][0]) / 2,
                                         (self.fl_wheel[0][1] + self.fl_wheel[2][1]) / 2])

    def get_rotation_matrix(self, angle: float) -> ndarray:

        return np.array([( cos(angle), sin(angle)),
                         (-sin(angle), cos(angle))])

    def transform(self, point: ndarray, angle_vector: ndarray, x: float, y: float) -> ndarray:

        # Rotational transform
        point = point.dot(angle_vector).T

        # Position translation
        point[0, :] += x
        point[1, :] += y
        
        return point

    def plot_car(self, x: float, y: float, yaw: float, steer: float) -> tuple[ndarray, ndarray, ndarray, ndarray, ndarray]:

        # Rotation matrices
        yaw_vector   = self.get_rotation_matrix(yaw)
        steer_vector = self.get_rotation_matrix(steer)

        fr_wheel = self.fr_wheel.copy()
        fl_wheel = self.fl_wheel.copy()

        # Rotate the wheels about its position
        fr_wheel -= self.fr_wheel_centre
        fl_wheel -= self.fl_wheel_centre
        fr_wheel  = fr_wheel@steer_vector
        fl_wheel  = fl_wheel@steer_vector
        fr_wheel += self.fr_wheel_centre
        fl_wheel += self.fl_wheel_centre

        outlines = self.transform(self.outlines, yaw_vector, x, y)
        fr_wheel = self.transform(fr_wheel, yaw_vector, x, y)
        fl_wheel = self.transform(fl_wheel, yaw_vector, x, y)
        rr_wheel = self.transform(self.wheel_format, yaw_vector, x, y)
        rl_wheel = self.transform(self.rl_wheel, yaw_vector, x, y)

        return outlines, fr_wheel, rr_wheel, fl_wheel, rl_wheel

def main():

    from matplotlib import pyplot as plt

    # Based on Tesla's model S 100D (https://www.car.info/en-se/tesla/model-s/model-s-100-kwh-awd-16457112/specs)
    overall_length = 4.97
    overall_width = 1.964
    tyre_diameter = 0.4826
    tyre_width = 0.2032
    axle_track = 1.662
    wheelbase = 2.96
    rear_overhang = (overall_length - wheelbase) / 2
    colour = 'black'

    desc = Description(overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase)
    desc_plots = desc.plot_car(30.0, -10.0, np.pi/4, np.deg2rad(25))
    
    ax = plt.axes()
    ax.set_aspect('equal')

    for desc_plot in desc_plots:
        ax.plot(*desc_plot, color=colour)

    plt.show()

if __name__ == '__main__':
    main()
