"""
B_spline curve generator helper
This FILE is part of multi-legged robot field exploration model

This programm is explained by roboLAND in university of southern california.
Please notify the source if you use it

Copyright(c) 2021-2025 Ryoma Liu
Email: shipengl@usc.edu
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si

# parameter
N = 3  # B Spline order

def bspline_planning(points, sn):
    x = []
    y = []
    for point in points:
        x.append(point[0])
        y.append(point[1])
    fit_points = []
    #print(points)
    if len(points) > 4:
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)
        x_list = list(x_tup)
        #xl = x.tolist()
        x_list[1] = x + [0.0, 0.0, 0.0, 0.0]
        y_list = list(y_tup)
        #yl = y.tolist()
        y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        for i in range(len(rx)):
            point = [rx[i],ry[i]]
            fit_points.append(point)
    else:
        print("please continue click, you must have more than 4 points to draw a B_spline")
    return rx, ry, fit_points

def main():
    print(__file__ + " start!!")
    # way points
    points = [[1,0],[2,1.2],[3,2],[4,1.2],[5,0]]
    print(points)
    x = []
    y = []
    for point in points:
        x.append(point[0])
        y.append(point[1])
    print(x)
    print(y)
    sn = 100  # sampling number

    rx, ry , fitpoints= bspline_planning(points, sn)

    # show results
    plt.plot(x, y, '-og', label="Waypoints")
    plt.plot(rx, ry, '-r', label="B-Spline path")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()