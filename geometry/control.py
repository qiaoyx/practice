import math
import numpy as np
import scipy as sp

import matplotlib as mp
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

import pandas as pd

from scipy import interpolate, optimize
from scipy.fftpack import fft, ifft, fftfreq, fftshift

# sigs = np.arange(-math.pi, math.pi, 0.01)
# sins = np.sin(sigs)

# vs = np.arange(-1.0, 1.0, 0.1)
# ws = np.arange(-math.pi, math.pi, 0.1)

vs = [1 for i in range(1, 100)]
ws = [math.sin(j / 10.0) for j in range(1, 100)]

xs = []
ys = []
__x = 0
__y = 0
for ii in range(len(vs)):
    for jj in range(len(vs)):
        _s = math.sin(ws[ii])
        _c = math.cos(ws[ii])
        __x += vs[ii] * _s
        __y += vs[ii] * _c
        xs.append(__x)
        ys.append(__y)
plt.plot(ys, xs)


# plt.plot(sins)
plt.show()
