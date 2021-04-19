import numpy as np

class Description:

    def __init__(self, length=0.0, width=0.0, rear2wheel=0.0, wheel_diameter=0.0, wheel_width=0.0, tread=0.0, wheelbase=0.0):

        self.length = length
        self.width = width
        self.rear2wheel = rear2wheel
        self.wheel_dia = wheel_diameter
        self.wheel_width = wheel_width
        self.tread = tread
        self.wheelbase = wheelbase

        
    def plot_car(self, x, y, yaw, steer=0.0, carcolor="black"):

        outline = np.array([[-self.rear2wheel, (self.length - self.rear2wheel), (self.length - self.rear2wheel), -self.rear2wheel, -self.rear2wheel],
                            [self.width / 2, self.width / 2, - self.width / 2, -self.width / 2, self.width / 2]])

        wheel_format = np.array([[self.wheel_dia, -self.wheel_dia, -self.wheel_dia, self.wheel_dia, self.wheel_dia],
                                [-self.wheel_width - self.tread, -self.wheel_width - self.tread, self.wheel_width - self.tread, self.wheel_width - self.tread, -self.wheel_width - self.tread]])

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
        fr_wheel[0, :] += self.wheelbase
        fl_wheel[0, :] += self.wheelbase

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

        outline_plot = (np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten())
        fr_plot = (np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten())
        rr_plot = (np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten())
        fl_plot = (np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten())
        rl_plot = (np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten())

        return outline_plot, fr_plot, rr_plot, fl_plot, rl_plot

def main():

    print("This script is not meant to be executable, and should be used as a library.")

if __name__ == '__main__':
    main()