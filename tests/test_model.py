# ruff: noqa: S101, PLR2004

from math import radians

from pytest import raises

from kbm import KinematicBicycleModel, NegativeValueError


def almost_equal(a: float, b: float, tolerance: float = 1e-6) -> bool:
    return abs(a - b) < tolerance


def test_zero_state(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=0.0, velocity=0.0, acceleration=0.0)

    assert state["x"] == 0.0
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == 0.0
    assert state["velocity"] == 0.0
    assert state["angular_velocity"] == 0.0


def test_velocity_x(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=0.0, velocity=10.0, acceleration=0.0)

    assert state["x"] == 0.5
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == 0.0
    assert state["velocity"] == 10.0
    assert state["angular_velocity"] == 0.0


def test_velocity_y(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=radians(90), steer=0.0, velocity=10.0, acceleration=0.0)

    assert almost_equal(state["x"], 0.0)
    assert state["y"] == 0.5
    assert state["yaw"] == radians(90)
    assert state["steer"] == 0.0
    assert state["velocity"] == 10.0
    assert state["angular_velocity"] == 0.0


def test_negative_velocity(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=0.0, velocity=-10.0, acceleration=0.0)

    assert state["x"] == -0.5
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == 0.0
    assert state["velocity"] == -10.0
    assert state["angular_velocity"] == 0.0


def test_acceleration(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=0.0, velocity=0.0, acceleration=5.0)

    assert state["x"] == 0.0125
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == 0.0
    assert state["velocity"] == 0.25
    assert state["angular_velocity"] == 0.0


def test_negative_acceleration(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=0.0, velocity=0.0, acceleration=-5.0)

    assert state["x"] == -0.0125
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == 0.0
    assert state["velocity"] == -0.25
    assert state["angular_velocity"] == 0.0


def test_yaw(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=radians(45), steer=0.0, velocity=10.0, acceleration=0.0)

    assert almost_equal(state["x"], state["y"])
    assert state["yaw"] == radians(45)
    assert state["steer"] == 0.0
    assert state["velocity"] == 10.0
    assert state["angular_velocity"] == 0.0


def test_steer_limits(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=radians(45), velocity=0.0, acceleration=0.0)

    assert state["x"] == 0.0
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == model.max_steer
    assert state["velocity"] == 0.0
    assert state["angular_velocity"] == 0.0


def test_negative_steer_limits(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=-radians(45), velocity=0.0, acceleration=0.0)

    assert state["x"] == 0.0
    assert state["y"] == 0.0
    assert state["yaw"] == 0.0
    assert state["steer"] == -model.max_steer
    assert state["velocity"] == 0.0
    assert state["angular_velocity"] == 0.0


def test_steer(model: KinematicBicycleModel) -> None:
    state = model.compute_state(x=0.0, y=0.0, yaw=0.0, steer=radians(30), velocity=10.0, acceleration=0.0)

    assert state["x"] == 0.4966703687246717
    assert state["y"] == 0.057606812365367445
    assert state["yaw"] == 0.11547005383792515
    assert state["steer"] == radians(30)
    assert state["velocity"] == 10.0
    assert state["angular_velocity"] == 2.309401076758503


def test_negative_wheelbase() -> None:
    with raises(NegativeValueError):
        KinematicBicycleModel(wheelbase=0.0, max_steer=0.0)
