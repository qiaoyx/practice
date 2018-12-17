# -*- coding: utf-8 -*-

import string
import math
import json
import copy
import pywt
import numpy as np
import scipy as sp
from scipy import interpolate, optimize
from imutils import paths

from sklearn.linear_model import Lasso
from sklearn.metrics import r2_score

import matplotlib.pyplot as plt
from scipy.fftpack import fft, ifft, fftfreq, fftshift


def autolabel(rects):
    """
    Attach a text label above each bar displaying its height
    """
    for rect in rects:
        height = rect.get_height()
        if height > 0.025:
            plt.text(rect.get_x() + rect.get_width()/2.,
                     1.01*height, '%f' % float(height),
                     ha='center', va='bottom', rotation=90)


def generate():
    data = []
    for i in range(1000):
        if i % 100 == 0:
            for j in range(10):
                data.append(math.cos(i*0.1))
            else:
                data.append(math.cos(i*0.1))


def readFile():
    lines = []
    with open("coords.json") as f:
        lines = f.readlines()
    elems = map(lambda x: json.loads(x), lines)
    return map(lambda kv: kv['x'], elems), map(lambda kv: kv['y'], elems)


def readFile1():
    lines = []
    _xs = []
    _ys = []
    with open("amcl_pose_0103.txt") as f:
        lines = f.readlines()
    lines.pop(0)
    for i in range(len(lines)):
        line = lines[i]
        if line is None:
            continue
        lst = map(string.strip, line.split(" "))
        _xs.append(float(lst[0]))
        _ys.append(float(lst[1]))
    return _xs, _ys


def smooth(l):
    if len(l) < 2:
        return l
    for i in range(1, len(l) - 1):
        l[i] = (l[i - 1] + 2 * l[i] + l[i + 1]) / 4
    return l


def difference(list):
    _init = list[:-1]
    _tail = list[1:]
    _ret = []
    for i in range(len(_init)):
        _ret.append(_tail[i] - _init[i])
    return _ret


def calc_theta(x1, y1, x2, y2):
    if x1 == 0 or y1 == 0 or x2 == 0 or y2 == 0:
        return 0, 0, 0

    theta1 = math.acos(x1 / math.sqrt(x1 * x1 + y1 * y1))
    theta2 = math.acos(x2 / math.sqrt(x2 * x2 + y2 * y2))
    dtheta = theta1 - theta2
    dx = x2 - x1*math.cos(dtheta) + y1*math.sin(dtheta)
    dy = y2 - x1*math.sin(dtheta) - y1*math.cos(dtheta)

    return dtheta, dx, dy


def transform(dx, dy, theta, x1, y1):
    x2 = x1 * math.cos(theta) - y1 * math.sin(theta) + dx
    y2 = x1 * math.sin(theta) + y1 * math.cos(theta) + dy
    return x2, y2


def MyFit(list, order=3):
    def polyfunc(coef, x):
        ret = 0
        L = len(coef) - 1
        for i in range(len(coef)):
            ret += coef[L - i] * math.pow(x, i)
        return ret

    _x = []
    for i in range(len(list)):
        _x.append(i)
    _y = np.polyfit(_x, list, order)
    _z = map(lambda x: polyfunc(_y, x), _x)
    return _z


def if_interact(f, d):
    fa = f[:-1]
    fb = f[1:]
    da = d[:-1]
    db = d[1:]
    for i in range(len(fa)):
        if (fa[i] - da[i]) * (fb[i] - db[i]) <= 0:
            f[i] = d[i]


def SVD_FFT(list, thres=1.0):
    _idx = []
    for i in range(len(list)):
        _idx.append(i)
    _fft = fft(list)
    # _frq = fftfreq(len(_fft))
    for i in range(len(_fft)):
        if abs(_fft[i]) < thres:
            _fft[i] = 0
    return ifft(_fft)


def mark(zero, zeroy):
    _dist = []
    _xs = []
    _ys = []
    for i in range(len(zero)):
        _xs.append(xs[zero[i] + 1])
        _ys.append(ys[zero[i] + 1])
    _xi = _xs[:-1]
    _xt = _xs[1:]
    _yi = _ys[:-1]
    _yt = _ys[1:]

#    plt.show()
#    plt.plot(_xs, _ys, 'bo--')
#    plt.show()

    for i in range(len(_xi)):
        __x = _xt[i] - _xi[i]
        __y = _yt[i] - _yi[i]
        _d = math.sqrt(__x * __x + __y * __y)

        _pi = []
        _pd = []
        if (_d > 6):
            _dist.append(_d)
            _pi.append(zero[i])
            _pi.append(zero[i + 1])
            _pd.append(zeroy[i])
            _pd.append(zeroy[i + 1])
            plt.plot(_pi, _pd, 'b-')

    return _dist, _xs, _ys


def stat(arr):
    # arr = map(lambda x: x * 100, arr)
    avgx = sum(arr) / len(arr)
    gtx = filter(lambda x: x > avgx, arr)
    ltx = filter(lambda x: x < avgx, arr)

    avgx = sum(gtx) / len(gtx)
    delt1 = map(lambda x: math.sqrt(math.pow(x - avgx, 2)), gtx)
    avgx = sum(ltx) / len(ltx)
    delt2 = map(lambda x: math.sqrt(math.pow(x - avgx, 2)), ltx)
    delt = delt1 + delt2
    delt = list(set(delt))
    return [i for i in range(len(delt))], delt


# xs, ys = readFile1()
xs, ys = readFile()
for i in range(len(xs)):
    x1 = xs[i]
    y1 = ys[i]
    x2, y2 = transform(17, -138, (49) * math.pi / 180, x1, y1)
    xs[i] = x2
    ys[i] = y2

dist = map(lambda x, y: math.sqrt(x*x + y*y), xs, ys)[1:]
dist = map(lambda x: x - dist[0], dist)
# dist = SVD_FFT(dist)
cA, cD = pywt.dwt(dist, 'haar')
# cA, cD = pywt.dwt(cA, 'haar')
# cA, cD = pywt.dwt(cA, 'haar')

fit = MyFit(dist)


dd = []
for i in range(len(cA)):
    dd.append(cA[i])
    dd.append(cA[i])
dd = smooth(dd)[1:]

dfit = MyFit(dd)
if_interact(dfit, dd)
dif = map(lambda x, y: abs(y - x), dfit, dd)
zd = []
for i in range(len(dif)):
    if dif[i] == 0:
        zd.append(i)

xxl = []
yyl = []
a = zd[:-1]
b = zd[1:]
for i in range(len(zd) - 1):
    xxl.append(a[i])
    yyl.append(0)
    tmp = 0
    for j in range(a[i], b[i]):
        if dif[j] > tmp:
            tmp = dif[j]
            xxl[i] = j
            yyl[i] = dist[j]

zero = []
zeroy = []
for i in range(len(cD)):
    if cD[i] == 0:
        zero.append(i * 2)
        zeroy.append(dist[i * 2])

plt.xlabel(r"X coordinates (m)")
plt.ylabel(r"Y coordinates (m)")
plt.title(r"Trajectory")
plt.grid(True)
plt.plot(xs, ys, 'g--.', linewidth=1)
plt.show()

plt.plot(dist, 'g.', linewidth=2)
plt.plot(zero, zeroy, 'r*')

marks, mxs, mys = mark(zero, zeroy)
plt.show()

avg = sum(marks) / len(marks)
marks = map(lambda x: math.sqrt(math.pow(x-avg, 2)), marks)
# print marks
# marks.sort()
# marks.reverse()
xx = [i for i in range(len(marks))]

# rect1 = plt.bar(np.arange(len(marks)), marks, _width, facecolor='blue')
# plt.show()

plt.grid(True)
plt.xlabel(r"Survey Times")
plt.ylabel(r"Standard Deviation Of X/Y coordinate (m)")
cx, cy = stat(mxs)
rect2 = plt.bar(cx, cy, .4, facecolor='green', label="X Bias")

cx, cy = stat(mys)
cx = [i+0.4 for i in cx]
rect3 = plt.bar(cx, cy, .4, facecolor='blue', label="Y Bias")
plt.legend(loc='upper right')
autolabel(rect2)
autolabel(rect3)
plt.show()
