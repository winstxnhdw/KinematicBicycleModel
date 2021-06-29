# Kinematic Bicycle Model
[![Total alerts](https://img.shields.io/lgtm/alerts/g/winstxnhdw/KinematicBicycleModel.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/winstxnhdw/KinematicBicycleModel/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/winstxnhdw/KinematicBicycleModel.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/winstxnhdw/KinematicBicycleModel/context:python)
<div align="center">
	<img src="resources/animation.gif" />
</div>
   
## Abstract
A python library for the Kinematic Bicycle model. The Kinematic Bicycle is a compromise between the non-linear and linear bicycle models for high-speed integration of the library with little configuration.

## Advantages
- The model allows the vehicle to come to rest without passing the model a negative acceleration; similar to the non-linear bicycle.
- This lightweight model is able to accurately represent a vehicle with no slip or tire stiffness.

```yaml
At initialisation
:param L:           (float) vehicle's wheelbase [m]
:param max_steer:   (float) vehicle's steering limits [rad]
:param dt:          (float) discrete time period [s]
:param c_r:         (float) vehicle's aerodynamic coefficient
:param c_a:         (float) vehicle's coefficient of resistance

Every frame
:param x:           (float) vehicle's x-coordinate [m]
:param y:           (float) vehicle's y-coordinate [m]
:param yaw:         (float) vehicle's heading [rad]
:param v:           (float) vehicle's velocity in the x-axis [m/s]
:param throttle:    (float) vehicle's accleration [m/s^2]
:param delta:       (float) vehicle's steering angle [rad]

:return x:          (float) vehicle's x-coordinate [m]
:return y:          (float) vehicle's y-coordinate [m]
:return yaw:        (float) vehicle's heading [rad]
:return v:          (float) vehicle's velocity in the x-axis [m/s]
:return delta:      (float) vehicle's steering angle [rad]
:return omega:      (float) vehicle's angular velocity [rad/s]
```

## Requirements
```bash
# Install NumPy
$ pip install numpy
```

## Demo
```bash
# Install requirements.txt
$ pip install -r requirements.txt

# Play the animation
$ python animation.py
```

## Concept
To simplify the equations, we perform all calculations from the rear axle.
<div align="center">
	<img src="resources/KinematicBicycleModel.png" />
</div>
