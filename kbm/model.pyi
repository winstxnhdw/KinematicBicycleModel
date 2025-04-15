from typing import (
    ReadOnly,  # pyright: ignore [reportAttributeAccessIssue]
    TypedDict,
)

class VehicleState(TypedDict):
    x: ReadOnly[float]
    y: ReadOnly[float]
    yaw: ReadOnly[float]
    steer: ReadOnly[float]
    velocity: ReadOnly[float]
    angular_velocity: ReadOnly[float]

class KinematicBicycleModel:
    wheelbase: float
    max_steer: float
    delta_time: float

    def __init__(self, *, wheelbase: float, max_steer: float, delta_time: float = 0.05) -> None:
        """
        Summary
        -------
        an implementation of the 2D Kinematic Bicycle Model for computing basic vehicle dynamics

        Attributes
        ----------
        wheelbase (float) : vehicle's wheelbase [m]
        max_steer (float) : vehicle's steering limits [rad]
        delta_time (float) : discrete time period [s]
        """

    def compute_state(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        steer: float,
        velocity: float,
        acceleration: float,
    ) -> VehicleState:
        """
        Summary
        -------
        computes the vehicle's state using the kinematic bicycle model

        Parameters
        ----------
        x (int) : vehicle's x-coordinate [m]
        y (int) : vehicle's y-coordinate [m]
        yaw (int) : vehicle's heading [rad]
        steer (int) : vehicle's steering angle [rad]
        velocity (int) : vehicle's velocity in the x-axis [m/s]
        acceleration (int) : vehicle's accleration [m/s^2]

        Returns
        -------
        vehicle_state (VehicleState) : updated vehicle state dictionary
        """
