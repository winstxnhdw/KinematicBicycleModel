# Kinematic Bicycle Model

[![python](https://img.shields.io/badge/python-3.9%20|%203.10%20|%203.11%20|%203.12%20|%203.13-blue)](https://www.python.org/)
![codeql.yml](https://github.com/winstxnhdw/KinematicBicycleModel/actions/workflows/codeql.yml/badge.svg)

<div align="center">
    <img src="resources/animation_wide.gif" />
</div>

## Abstract

A python library for the Kinematic Bicycle model. The model can be defined with the following state-space representation,

$$
\frac{d}{dt}
\begin{pmatrix}
x \\
y \\
\theta \\
v
\end{pmatrix} =
\begin{pmatrix}
v\cos{\theta} \\
v\sin{\theta} \\
\frac{v\tan{\delta}}{L} \\
a
\end{pmatrix}
$$

where $v$ is the vehicle's velocity in the x-axis, $\theta$ is the vehicle's yaw, $\delta$ is the steering angle, $L$ is the vehicle's wheelbase, $a$ is the acceleration/throttle, $f$ is friction in the x-axis.

## Installation

```bash
pip install git+https://github.com/winstxnhdw/KinematicBicycleModel
```

## Usage

The model is written entirely in [Cython](https://cython.org) and is completely typesafe. The following code sample demonstrates how to use the model.

```python
from kbm import KinematicBicycleModel

model = KinematicBicycleModel(wheelbase=2.96, max_steer=0.57596, delta_time=0.05)
state = model.update(
    x=0.0,
    y=0.0,
    yaw=0.0,
    steer=0.0,
    velocity=0.0,
    acceleration=5.0
)

print(f"The vehicle is at ({state['x']}, {state['y']})")
print(f"The vehicle is facing {state['yaw']} rad")
print(f"The vehicle is steering at {state['steer']} rad")
print(f"The vehicle is moving at {state['velocity']} m/s")
print(f"The vehicle is turning at {state['angular_velocity']} rad/s")
```

## Limitations

Just like with all other bicycle models, this model is a discrete model and loses its accuracy when the time step is set too large or the vehicle is made to travel at unreasonably high speeds. Usually, the FPS of the simulation should be set to the highest possible value for the greatest accuracy. However, for rendering high-quality GIFs, 50 FPS is found to be most optimal. See the [GIF89a specification](https://www.w3.org/Graphics/GIF/spec-gif89a.txt).

## Demo

Recursively git clone the repository

```bash
git clone --recursive https://github.com/winstxnhdw/KinematicBicycleModel.git
```

Play the animation

```bash
uv run animate.py
```

## Concept

Though our implementation is titled the `Kinematic Bicycle Model`, it does take into account some forward friction. This was never intended to improve the accuracy of the model. Instead, it provides a more intuitive API by removing the need to constantly control the input throttle. However, that is where the differences end. You can read about the bicycle model in full detail by Theers et al., [here](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html).
