from math import radians

from pytest import fixture

from kbm import KinematicBicycleModel


@fixture
def model() -> KinematicBicycleModel:
    return KinematicBicycleModel(wheelbase=2.5, max_steer=radians(30))
