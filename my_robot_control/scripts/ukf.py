#!/usr/bin/env python

import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import sympy

from IPython.display import display
sympy.init_printing(use_latex='mathjax')
x, x_vel, y = sympy.symbols('x, x_vel y')
H = sympy.Matrix([sympy.sqrt(x**2 + y**2)])
state = sympy.Matrix([x, x_vel, y])
J = H.jacobian(state)
display(state)
display(J)

dt = 1.0

def f_cv(x, dt):
    F = np.array([[1, dt, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, dt],
                  [0, 0, 0, 1]])
    return np.dot(F, x)

def h_cv(x):
    return x[0, 2]

def sigma_points(self, x, P):
    pass

sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=1.)
ukf = UKF(dim_x=4, dim_z=2, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)

ukf.x = np.array([0., 0., 0., 0.])
ukf.R = np.diag([0.3**2, 0.3**2])


