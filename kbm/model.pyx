# cython: language_level=3, cdivision=True

from libc.math cimport atan2, cos, sin, tan
from kbm.exceptions import NegativeValueError

cdef packed struct VehicleState:
    double x
    double y
    double yaw
    double steer
    double velocity
    double angular_velocity


cdef class KinematicBicycleModel:
    cdef readonly double wheelbase
    cdef readonly double max_steer
    cdef readonly double delta_time

    def __cinit__(self, *, const long double wheelbase, const long double max_steer, const long double delta_time=0.05):
        if delta_time <= 0:
            raise NegativeValueError("delta_time")

        if wheelbase <= 0:
            raise NegativeValueError("wheelbase")

        self.delta_time = delta_time
        self.wheelbase = wheelbase
        self.max_steer = max_steer

    cpdef const VehicleState compute_state(
        self,
        const double x,
        const double y,
        const double yaw,
        const double steer,
        const double velocity,
        const double acceleration
    ) noexcept:
        cdef double new_velocity
        cdef double new_steer
        cdef double angular_velocity
        cdef double new_yaw
        cdef VehicleState state

        with nogil:
            new_velocity = velocity + self.delta_time * acceleration
            new_steer = self.max_steer if steer > self.max_steer else -self.max_steer if steer < -self.max_steer else steer
            angular_velocity = new_velocity * tan(new_steer) / self.wheelbase
            new_yaw = yaw + angular_velocity * self.delta_time

            state.x = x + new_velocity * cos(new_yaw) * self.delta_time
            state.y = y + new_velocity * sin(new_yaw) * self.delta_time
            state.yaw = atan2(sin(new_yaw), cos(new_yaw))
            state.steer = new_steer
            state.velocity = new_velocity
            state.angular_velocity = angular_velocity

        return state
