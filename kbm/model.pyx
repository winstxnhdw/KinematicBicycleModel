# cython: language_level=3, cdivision=True, cpow=True, auto_pickle=False, binding=False

from libc.math cimport atan2, cos, sin, tan
from kbm.exceptions import NegativeValueError

cdef packed struct VehicleState:
    double x
    double y
    double yaw
    double steer
    double velocity
    double angular_velocity


cdef inline const VehicleState compute_vehicle_state(
    const double x,
    const double y,
    const double yaw,
    const double steer,
    const double velocity,
    const double acceleration,
    const double wheelbase,
    const double max_steer,
    const double delta_time,
) noexcept nogil:
    cdef VehicleState state

    new_velocity = velocity + delta_time * acceleration
    new_steer = max_steer if steer > max_steer else -max_steer if steer < -max_steer else steer
    angular_velocity = new_velocity * tan(new_steer) / wheelbase
    new_yaw = yaw + angular_velocity * delta_time
    velocity_delta = new_velocity * delta_time
    cos_new_yaw = cos(new_yaw)
    sin_new_yaw = sin(new_yaw)

    state.x = x + velocity_delta * cos_new_yaw
    state.y = y + velocity_delta * sin_new_yaw
    state.yaw = atan2(sin_new_yaw, cos_new_yaw)
    state.steer = new_steer
    state.velocity = new_velocity
    state.angular_velocity = angular_velocity

    return state


cdef class KinematicBicycleModel:
    cdef readonly double wheelbase
    cdef readonly double max_steer
    cdef readonly double delta_time

    def __cinit__(self, *, const long double wheelbase, const long double max_steer, const long double delta_time=0.05):
        if wheelbase <= 0:
            raise NegativeValueError("wheelbase")

        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.delta_time = delta_time

    cpdef const VehicleState compute_state(
        self,
        const double x,
        const double y,
        const double yaw,
        const double steer,
        const double velocity,
        const double acceleration
    ) noexcept:
        return compute_vehicle_state(
            x,
            y,
            yaw,
            steer,
            velocity,
            acceleration,
            self.wheelbase,
            self.max_steer,
            self.delta_time,
        )


    cpdef const VehicleState compute_state_nogil(
        self,
        const double x,
        const double y,
        const double yaw,
        const double steer,
        const double velocity,
        const double acceleration
    ) noexcept:
        with nogil:
            return compute_vehicle_state(
                x,
                y,
                yaw,
                steer,
                acceleration,
                velocity,
                self.max_steer,
                self.wheelbase,
                self.delta_time,
            )
