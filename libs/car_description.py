import numpy as np

class Description:

    def __init__(self, overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase):
        
        """
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
        
        :return outlines:           (list) vehicle's outlines [x, y]
        :return fr_wheel:           (list) vehicle's front-right axle [x, y]
        :return rr_wheel:           (list) vehicle's rear-right axle [x, y]
        :return fl_wheel:           (list) vehicle's front-left axle [x, y]
        :return rl_wheel:           (list) vehicle's rear-right axle [x, y]
        """

        centre_to_wheel = axle_track / 2
        centre_to_side = overall_width / 2
        rear_axle_to_front_bumper = overall_length - rear_overhang

        vehicle_vertices = np.array([(-rear_overhang,              centre_to_side),
                                     ( rear_axle_to_front_bumper,  centre_to_side),
                                     ( rear_axle_to_front_bumper, -centre_to_side),
                                     (-rear_overhang,             -centre_to_side)])

        wheel_vertices = np.array([(-tyre_diameter,  tyre_width - centre_to_wheel),
                                   ( tyre_diameter,  tyre_width - centre_to_wheel),
                                   ( tyre_diameter, -tyre_width - centre_to_wheel),
                                   (-tyre_diameter, -tyre_width - centre_to_wheel)])

        self.outlines = np.concatenate((vehicle_vertices, [vehicle_vertices[0]]))

        self.wheel_format = np.concatenate((wheel_vertices, [wheel_vertices[0]]))
        self.rl_wheel = np.copy(self.wheel_format)
        self.rl_wheel[:,1] *= -1
        self.fl_wheel = np.copy(self.rl_wheel)
        self.fl_wheel[:, 0] += wheelbase 
        self.fr_wheel = np.copy(self.wheel_format)
        self.fr_wheel[:, 0] += wheelbase
                                   
        self.fr_wheel_centre = np.array([(self.fr_wheel[0][0] + self.fr_wheel[2][0]) / 2,
                                         (self.fr_wheel[0][1] + self.fr_wheel[2][1]) / 2])

        self.fl_wheel_centre = np.array([(self.fl_wheel[0][0] + self.fl_wheel[2][0]) / 2,
                                         (self.fl_wheel[0][1] + self.fl_wheel[2][1]) / 2])

    def get_rotation_matrix(self, angle):

        return np.array([( np.cos(angle), np.sin(angle)),
                         (-np.sin(angle), np.cos(angle))])

    def transform(self, point, x, y, angle_vector):

        # Rotational transform
        point = point.dot(angle_vector).T

        # Position translation
        point[0,:] += x
        point[1,:] += y
        
        return point

    def plot_car(self, x, y, yaw, steer=0.0):

        # Rotation matrices
        yaw_vector   = self.get_rotation_matrix(yaw)
        steer_vector = self.get_rotation_matrix(steer)

        fr_wheel = np.copy(self.fr_wheel)
        fl_wheel = np.copy(self.fl_wheel)

        # Rotate the wheels about its position
        fr_wheel -= self.fr_wheel_centre
        fl_wheel -= self.fl_wheel_centre
        fr_wheel  = fr_wheel.dot(steer_vector)
        fl_wheel  = fl_wheel.dot(steer_vector)
        fr_wheel += self.fr_wheel_centre
        fl_wheel += self.fl_wheel_centre

        outlines = self.transform(self.outlines, x, y, yaw_vector)
        fr_wheel = self.transform(fr_wheel, x, y, yaw_vector)
        fl_wheel = self.transform(fl_wheel, x, y, yaw_vector)
        rr_wheel = self.transform(self.wheel_format, x, y, yaw_vector)
        rl_wheel = self.transform(self.rl_wheel, x, y, yaw_vector)

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