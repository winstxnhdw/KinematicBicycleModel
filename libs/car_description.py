import numpy as np
import matplotlib.pyplot as plt

class Description:

    def __init__(self):

        self.L = 4.5
        self.width = 2.0
        self.rear2wheel = 1.0
        self.wheel_dia = 0.15 * 2
        self.wheel_width = 0.2
        self.tread = 0.7
        self.wheelbase = 2.5

        
def plot_car(x, y, yaw, steer=0.0, carcolor="black"):

    desc = Description()

    outline = np.array([[-desc.rear2wheel, (desc.L - desc.rear2wheel), (desc.L - desc.rear2wheel), -desc.rear2wheel, -desc.rear2wheel],
                        [desc.width / 2, desc.width / 2, - desc.width / 2, -desc.width / 2, desc.width / 2]])

    wheel_format = np.array([[desc.wheel_dia, -desc.wheel_dia, -desc.wheel_dia, desc.wheel_dia, desc.wheel_dia],
                            [-desc.wheel_width - desc.tread, -desc.wheel_width - desc.tread, desc.wheel_width - desc.tread, desc.wheel_width - desc.tread, -desc.wheel_width - desc.tread]])

    fr_wheel = np.copy(wheel_format)
    rr_wheel = np.copy(wheel_format)
    fl_wheel = np.copy(wheel_format)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    rot1 = np.array([[np.cos(yaw), np.sin(yaw)],
                     [-np.sin(yaw), np.cos(yaw)]])
    rot2 = np.array([[np.cos(steer), np.sin(steer)],
                     [-np.sin(steer), np.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(rot2)).T
    fl_wheel = (fl_wheel.T.dot(rot2)).T
    fr_wheel[0, :] += desc.wheelbase
    fl_wheel[0, :] += desc.wheelbase

    fr_wheel = (fr_wheel.T.dot(rot1)).T
    fl_wheel = (fl_wheel.T.dot(rot1)).T

    outline = (outline.T.dot(rot1)).T
    rr_wheel = (rr_wheel.T.dot(rot1)).T
    rl_wheel = (rl_wheel.T.dot(rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), carcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), carcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), carcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), carcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), carcolor)
             
    plt.plot(x, y, "+", color=carcolor, markersize=2)

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == '__main__':
    main()