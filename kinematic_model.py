from math import cos, sin, tan

from libs import normalise_angle


class KinematicBicycleModel:
    """
    Summary
    -------
    This class implements the 2D Kinematic Bicycle Model for vehicle dynamics

    Attributes
    ----------
    dt (float) : discrete time period [s]
    wheelbase (float) : vehicle's wheelbase [m]
    max_steer (float) : vehicle's steering limits [rad]

    Methods
    -------
    __init__(wheelbase: float, max_steer: float, delta_time: float=0.05)
        initialises the class

    update(x, y, yaw, velocity, acceleration, steering_angle)
        updates the vehicle's state using the kinematic bicycle model
    """
    def __init__(self, wheelbase: float, max_steer: float, delta_time: float=0.05):

        self.delta_time = delta_time
        self.wheelbase = wheelbase
        self.max_steer = max_steer


    def update(self, x: float, y: float, yaw: float, velocity: float, acceleration: float, steering_angle: float) -> tuple[float, ...]:
        """
        Summary
        -------
        Updates the vehicle's state using the kinematic bicycle model

        Parameters
        ----------
        x (int) : vehicle's x-coordinate [m]
        y (int) : vehicle's y-coordinate [m]
        yaw (int) : vehicle's heading [rad]
        velocity (int) : vehicle's velocity in the x-axis [m/s]
        acceleration (int) : vehicle's accleration [m/s^2]
        steering_angle (int) : vehicle's steering angle [rad]

        Returns
        -------
        new_x (int) : vehicle's x-coordinate [m]
        new_y (int) : vehicle's y-coordinate [m]
        new_yaw (int) : vehicle's heading [rad]
        new_velocity (int) : vehicle's velocity in the x-axis [m/s]
        steering_angle (int) : vehicle's steering angle [rad]
        angular_velocity (int) : vehicle's angular velocity [rad/s]
        """
        # Compute the local velocity in the x-axis
        new_velocity = velocity + self.delta_time * acceleration

        # Limit steering angle to physical vehicle limits
        steering_angle = -self.max_steer if steering_angle < -self.max_steer else self.max_steer if steering_angle > self.max_steer else steering_angle

        # Compute the angular velocity
        angular_velocity = new_velocity*tan(steering_angle) / self.wheelbase

        # Compute the final state using the discrete time model
        new_x   = x + velocity*cos(yaw)*self.delta_time
        new_y   = y + velocity*sin(yaw)*self.delta_time
        new_yaw = normalise_angle(yaw + angular_velocity*self.delta_time)
        
        return new_x, new_y, new_yaw, new_velocity, steering_angle, angular_velocity
