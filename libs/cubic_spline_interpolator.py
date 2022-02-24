import numpy as np

from numpy import ndarray
from numpy.typing import ArrayLike
from scipy.interpolate import CubicSpline

def initialise_cubic_spline(x: ArrayLike, y: ArrayLike, ds: ArrayLike, bc_type: str) -> tuple[CubicSpline, ndarray]:

    distance = np.concatenate((np.zeros(1), np.cumsum(np.hypot(np.ediff1d(x), np.ediff1d(y)))))
    points = np.array([x, y]).T
    s = np.arange(0, distance[-1], ds)
    cs = CubicSpline(distance, points, bc_type=bc_type, axis=0, extrapolate=False)
    return cs, s

def generate_cubic_spline(x: ArrayLike, y: ArrayLike, ds: float=0.05, bc_type: str='natural') -> tuple[ndarray, ndarray, ndarray, ndarray]:
    
    cs, s = initialise_cubic_spline(x, y, ds, bc_type)

    dx, dy = cs.derivative(1)(s).T
    yaw = np.arctan2(dy, dx)

    ddx, ddy = cs.derivative(2)(s).T
    curvature = (ddy*dx - ddx*dy) / ((dx*dx + dy*dy)**1.5)

    cx, cy = cs(s).T
    return cx, cy, yaw, curvature

def generate_cubic_path(x: ArrayLike, y: ArrayLike, ds: float=0.05, bc_type: str='natural') -> tuple[ndarray, ndarray]:

    cs, s = initialise_cubic_spline(x, y, ds, bc_type)
    cx, cy = cs(s).T
    return cx, cy
    
def calculate_spline_yaw(x: ArrayLike, y: ArrayLike, ds: float=0.05, bc_type: str='natural') -> ndarray:
    
    cs, s = initialise_cubic_spline(x, y, ds, bc_type)
    dx, dy = cs.derivative(1)(s).T
    return np.arctan2(dy, dx)

def calculate_spline_curvature(x: ArrayLike, y: ArrayLike, ds: float=0.05, bc_type: str='natural') -> ndarray:

    cs, s = initialise_cubic_spline(x, y, ds, bc_type)
    dx, dy = cs.derivative(1)(s).T
    ddx, ddy = cs.derivative(2)(s).T
    return (ddy*dx - ddx*dy) / ((dx*dx + dy*dy)**1.5)

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
