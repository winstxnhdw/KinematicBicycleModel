# Kinematic Bicycle Model

[![Total alerts](https://img.shields.io/lgtm/alerts/g/winstxnhdw/KinematicBicycleModel.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/winstxnhdw/KinematicBicycleModel/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/winstxnhdw/KinematicBicycleModel.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/winstxnhdw/KinematicBicycleModel/context:python)

<div align="center">
	<img src="resources/animation_wide.gif" />
</div>
   
## Abstract

A python library for the Kinematic Bicycle model. The Kinematic Bicycle is a compromise between the non-linear and linear bicycle models for high-speed integration of the library with little configuration.

```yaml
At initialisation
:param wheelbase:       (float) vehicle's wheelbase [m]
:param max_steer:       (float) vehicle's steering limits [rad]
:param dt:              (float) discrete time period [s]
:param c_r:             (float) vehicle's coefficient of resistance 
:param c_a:             (float) vehicle's aerodynamic coefficient

At every time step  
:param x:               (float) vehicle's x-coordinate [m]
:param y:               (float) vehicle's y-coordinate [m]
:param yaw:             (float) vehicle's heading [rad]
:param velocity:        (float) vehicle's velocity in the x-axis [m/s]
:param throttle:        (float) vehicle's accleration [m/s^2]
:param delta:           (float) vehicle's steering angle [rad]

:return x:              (float) vehicle's x-coordinate [m]
:return y:              (float) vehicle's y-coordinate [m]
:return yaw:            (float) vehicle's heading [rad]
:return velocity:       (float) vehicle's velocity in the x-axis [m/s]
:return delta:          (float) vehicle's steering angle [rad]
:return omega:          (float) vehicle's angular velocity [rad/s]
```

## Advantages

- The model allows the vehicle to come to rest without passing the model a negative acceleration; similar to the non-linear bicycle.
- This lightweight model is able to accurately represent a vehicle with no slip or tire stiffness.

## Limitations

Just like with all other bicycle models, this model is a discrete model and loses its accuracy when the time step is set too large or the vehicle is made to travel at unreasonably high speeds. Usually, the FPS of the simulation should be set to the highest possible value for the greatest accuracy. However, for rendering high-quality GIFs, 50 FPS is found to be most optimal.

## Requirements

Supports Python 3.10 or greater.

```bash
pip install numpy
```

## Demo

Install the requirements

```bash
pip install -r requirements.txt
```

Play the animation

```bash
python animation.py
```

## Concept

To simplify the equations, we perform all calculations from the rear axle.

<div align="center">
	<img src="resources/KinematicBicycleModel.png" />
</div>
