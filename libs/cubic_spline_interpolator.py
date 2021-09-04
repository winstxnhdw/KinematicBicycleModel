import numpy as np

from scipy.interpolate import CubicSpline

def initialise_cubic_spline(x, y, ds, bc_type):

    distance = np.concatenate(([0], np.cumsum(np.hypot(np.ediff1d(x), np.ediff1d(y)))))
    s = np.arange(0, distance[-1], ds)
    points = np.array([x, y]).T
    cs = CubicSpline(distance, points, bc_type=bc_type, axis=0, extrapolate=False)

    return cs, s

def generate_cubic_spline(x, y, ds=0.05, bc_type='natural'):
    
    cs, s = initialise_cubic_spline(x, y, ds, bc_type)

    # dx = dcs[0],  dy = dcs[1], ddx = ddcs[0],  ddy = ddcs[1]
    dcs = cs.derivative(1)(s).T
    yaw = np.arctan2(dcs[1], dcs[0])

    ddcs = cs.derivative(2)(s).T
    curvature = (ddcs[1]*dcs[0] - ddcs[0]*dcs[1]) / ((dcs[0]*dcs[0] + dcs[1]*dcs[1])**1.5)

    cs_points = cs(s).T

    return cs_points[0], cs_points[1], yaw, curvature

def generate_cubic_path(x, y, ds=0.05, bc_type='natural'):

    cs, s = initialise_cubic_spline(x, y, ds, bc_type)
    cs_points = cs(s).T
    return cs_points[0], cs_points[1]

def calculate_spline_yaw(x, y, ds=0.05, bc_type='natural'):
    
    cs, s = initialise_cubic_spline(x, y, ds, bc_type)
    dcs = cs.derivative(1)(s).T
    return np.arctan2(dcs[1], dcs[0])

def calculate_spline_curvature(x, y, ds=0.05, bc_type='natural'):

    cs, s = initialise_cubic_spline(x, y, ds, bc_type)
    dcs = cs.derivative(1)(s).T
    ddcs = cs.derivative(2)(s).T
    return (ddcs[1]*dcs[0] - ddcs[0]*dcs[1]) / ((dcs[0]*dcs[0] + dcs[1]*dcs[1])**1.5)

def main():
    
    import pandas as pd
    from matplotlib import pyplot as plt

    dir_path = 'waypoints.csv'
    df = pd.read_csv(dir_path)
    x = df['x'].values
    y = df['y'].values

    px, py = generate_cubic_path(x, y)
    pyaw = calculate_spline_yaw(x, y)
    pk = calculate_spline_curvature(x, y)

    fig, ax = plt.subplots(1, 3, figsize=(15, 5))
    plt.style.use('seaborn-pastel')

    ax[0].set_box_aspect(1)
    ax[0].set_title('Geometry')
    ax[0].plot(px, py, c='m')

    ax[1].set_box_aspect(1)
    ax[1].set_title('Yaw')
    ax[1].plot(pyaw, c='m')

    ax[2].set_box_aspect(1)
    ax[2].set_title('Curvature')
    ax[2].plot(pk, c='m')
    
    plt.show()

if __name__ == '__main__':
    main()