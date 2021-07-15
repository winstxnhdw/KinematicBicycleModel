import math as m

def normalise_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    angle = m.atan2(m.sin(angle), m.cos(angle))

    return angle