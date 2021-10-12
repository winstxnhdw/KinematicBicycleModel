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
        
        :return outline:            (list) vehicle's outline [x, y]
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
                                   
        self.wheelbase = wheelbase
        self.outline = np.concatenate((vehicle_vertices, [vehicle_vertices[0]]))
        self.wheel_format = np.concatenate((wheel_vertices, [wheel_vertices[0]]))
        self.wheel_centre = np.array([(wheel_vertices[0][0] + wheel_vertices[2][0]) / 2,
                                      (wheel_vertices[0][1] + wheel_vertices[2][1]) / 2])

    def plot_car(self, x, y, yaw, steer=0.0):

        # Rotation matrices
        yaw_vector = np.array([( np.cos(yaw), np.sin(yaw)),
                               (-np.sin(yaw), np.cos(yaw))])
                              
        steer_vector = np.array([( np.cos(steer), np.sin(steer)),
                                 (-np.sin(steer), np.cos(steer))])

        outline = np.copy(self.outline)
        fr_wheel = np.copy(self.wheel_format)
        rr_wheel = np.copy(self.wheel_format)
        fl_wheel = np.copy(self.wheel_format)
        rl_wheel = np.copy(self.wheel_format)

        fl_wheel[:,1] *= -1
        rl_wheel[:,1] *= -1

        # Rotate the wheels about its position
        fr_wheel -= self.wheel_centre
        fl_wheel += self.wheel_centre
        fr_wheel = fr_wheel.dot(steer_vector)
        fl_wheel = fl_wheel.dot(steer_vector)
        fr_wheel += self.wheel_centre
        fl_wheel -= self.wheel_centre

        fr_wheel[:,0] += self.wheelbase
        fl_wheel[:,0] += self.wheelbase

        # Rotational transformation
        outline = (outline.dot(yaw_vector)).T
        fr_wheel = (fr_wheel.dot(yaw_vector)).T
        fl_wheel = (fl_wheel.dot(yaw_vector)).T
        rr_wheel = (rr_wheel.dot(yaw_vector)).T
        rl_wheel = (rl_wheel.dot(yaw_vector)).T

        # Translate the car
        outline[0, :] += x
        outline[1, :] += y

        fr_wheel[0, :] += x
        fr_wheel[1, :] += y

        rr_wheel[0, :] += x
        rr_wheel[1, :] += y

        fl_wheel[0, :] += x
        fl_wheel[1, :] += y

        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        return outline, fr_wheel, rr_wheel, fl_wheel, rl_wheel

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
    outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = desc.plot_car(30.0, -10.0, np.pi/4, np.deg2rad(25))
    
    ax = plt.axes()
    ax.set_aspect('equal')

    ax.plot(*outline_plot, color=colour)
    ax.plot(*fr_plot, color=colour)
    ax.plot(*rr_plot, color=colour)
    ax.plot(*fl_plot, color=colour)
    ax.plot(*rl_plot, color=colour)

    plt.show()

if __name__ == '__main__':
    main()