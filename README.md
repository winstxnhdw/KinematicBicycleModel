# Kinematic Bicycle Model

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

```yaml
At initialisation
:param wheelbase:           (float) vehicle's wheelbase [m]
:param max_steer:           (float) vehicle's steering limits [rad]
:param delta_time:          (float) discrete time period [s]

At every time step  
:param x:                   (float) vehicle's x-coordinate [m]
:param y:                   (float) vehicle's y-coordinate [m]
:param yaw:                 (float) vehicle's heading [rad]
:param velocity:            (float) vehicle's velocity in the x-axis [m/s]
:param acceleration:        (float) vehicle's accleration [m/s^2]
:param steering_angle:      (float) vehicle's steering angle [rad]

:return x:                  (float) vehicle's x-coordinate [m]
:return y:                  (float) vehicle's y-coordinate [m]
:return yaw:                (float) vehicle's heading [rad]
:return velocity:           (float) vehicle's velocity in the x-axis [m/s]
:return steering_angle:     (float) vehicle's steering angle [rad]
:return angular_velocity:   (float) vehicle's angular velocity [rad/s]
```

## Limitations

Just like with all other bicycle models, this model is a discrete model and loses its accuracy when the time step is set too large or the vehicle is made to travel at unreasonably high speeds. Usually, the FPS of the simulation should be set to the highest possible value for the greatest accuracy. However, for rendering high-quality GIFs, 50 FPS is found to be most optimal. See the [GIF89a specification](https://www.w3.org/Graphics/GIF/spec-gif89a.txt).

## Requirements

The model itself has zero dependencies and can run its fastest on vanilla python. However, to support type-hinting, Python 3.10 or greater is required.

## Demo

Recursively git clone the repository

```bash
git clone --recursive https://github.com/winstxnhdw/KinematicBicycleModel.git
```

Install the requirements

```bash
pip install -r requirements.txt
```

Play the animation

```bash
python animate.py
```

## Concept

Though our implementation is titled the `Kinematic Bicycle Model`, it does take into account some forward friction. This was never intended to improve the accuracy of the model. Instead, it provides a more intuitive API by removing the need to constantly control the input throttle. However, that is where the differences end. You can read about the bicycle model in full detail by Theers et al., [here](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html).
