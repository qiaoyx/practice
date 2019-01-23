import math
import numpy as np
from scipy.optimize import leastsq
import pylab as pl


def func(x, p):
    A, k, theta = p
    return A * np.sin(2 * np.pi * k * x + theta)


def residuals(p, y, x):
    return y - func(x, p)
