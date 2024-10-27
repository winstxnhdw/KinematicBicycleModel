# cython: language_level=3, cdivision=True

from libc.math cimport atan2, cos, sin, tan


cdef packed struct VehicleState:
    long double x
    long double y
    long double yaw
    long double steer
    long double velocity
    long double angular_velocity


cdef class KinematicBicycleModel:
    cdef long double wheelbase
    cdef long double max_steer
    cdef long double delta_time

    def __init__(self, const long double wheelbase, const long double max_steer, const long double delta_time=0.05):
        if delta_time <= 0:
            raise ValueError("`delta_time` must be positive")

        if wheelbase <= 0:
            raise ValueError("`wheelbase` must be positive")

        self.delta_time = delta_time
        self.wheelbase = wheelbase
        self.max_steer = max_steer

    cpdef const VehicleState update(
        self,
        const long double x,
        const long double y,
        const long double yaw,
        const long double steer,
        const long double velocity,
        const long double acceleration
    ):
        cdef long double new_velocity = velocity + self.delta_time * acceleration
        cdef long double new_steer = self.max_steer if steer > self.max_steer else -self.max_steer if steer < -self.max_steer else steer
        cdef long double angular_velocity = new_velocity * tan(new_steer) / self.wheelbase
        cdef long double new_yaw = yaw + angular_velocity * self.delta_time
        cdef VehicleState state
        state.x = x + velocity * cos(new_yaw) * self.delta_time
        state.y = y + velocity * sin(new_yaw) * self.delta_time
        state.yaw = atan2(sin(new_yaw), cos(new_yaw))
        state.steer = new_steer
        state.velocity = new_velocity
        state.angular_velocity = angular_velocity

        return state
