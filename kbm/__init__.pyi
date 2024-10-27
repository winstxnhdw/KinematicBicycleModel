from typing import TypedDict

from typing_extensions import ReadOnly

class VehicleState(TypedDict):
    x: ReadOnly[float]
    y: ReadOnly[float]
    yaw: ReadOnly[float]
    steer: ReadOnly[float]
    velocity: ReadOnly[float]
    angular_velocity: ReadOnly[float]

class KinematicBicycleModel:
    def __init__(self, wheelbase: float, max_steer: float, delta_time: float = 0.05) -> None: ...
    def update(
        self,
        x: float,
        y: float,
        yaw: float,
        steer: float,
        velocity: float,
        acceleration: float,
    ) -> VehicleState: ...
