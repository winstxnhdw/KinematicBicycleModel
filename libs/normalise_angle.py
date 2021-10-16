from math import atan2, sin, cos

"""
:param angle:       (float) angle [rad]

:return angle:      (float) angle [rad]
"""

normalise_angle = lambda angle : atan2(sin(angle), cos(angle))
