# coding=utf-8
import math
import cPickle
import numpy as np
import matplotlib.pyplot as plt

cell = 0.25 * math.pi / 180.0
angle_table = map(lambda i: float((448 - i)) * cell, xrange(897))
ctable = map(lambda t: math.cos(t), angle_table)
stable = map(lambda t: math.sin(t), angle_table)


def dot(v):
    return np.dot(v, v.T)


def norm(v):
    return np.linalg.norm(v)


def shelf(msg):
    _ranges = np.array(_msg['ranges'])
    for i in range(len(_ranges)):
        if _ranges[i] < 0.1 or _ranges[i] > 35.0:
            _ranges[i] = 0
        else:
            _ranges[i] = _ranges[i] * 100

    xs = map(lambda d, t: d*t, _ranges, stable)
    ys = map(lambda d, t: d*t, _ranges, ctable)

    def x(i):
        return _ranges[i]*stable[i]

    def y(i):
        return _ranges[i]*ctable[i]

    def xy(s, e):
        x, y = 0, 0
        for i in range(s, e+1):
            x, y = x + xs[i], y + ys[i]
        return (x/(e-s+1), y/(e-s+1))

    segments, s, e, d = [], 0, 0, 0
    minx, miny, maxx, maxy = 9999, 9999, 0, 0
    for i in range(len(_ranges)):
        if _ranges[i] == 0:
            continue
        minx, miny = min(minx, xs[i]), min(miny, ys[i])
        maxx, maxy = max(maxx, xs[i]), max(maxy, ys[i])
        _d = abs(xs[i] - x(e)) + abs(ys[i] - y(e))
        __d = maxx + maxy - minx - miny
        if _d > 10 or __d > 15:
            if __d > 3 and (e-s > 2):
                segments.append((s, e, d))
            minx, miny, maxx, maxy = 9999, 9999, 0, 0
            s, e, d = i, i, 0
        else:
            e, d = i, __d

    plt.plot(xs, ys, 'y.')
    xxs, yys = [], []
    for seg in segments:
        _x, _y = xy(seg[0], seg[1])
        xxs.append(_x)
        yys.append(_y)

        if seg[2] > 7:
            plt.plot([_x], [_y], 'r*')
        else:
            plt.plot([_x], [_y], 'k*')

    minL, maxL = 71, 94
    for i in range(len(xxs)):
        if _ranges[i] == 0:
            continue
        _n = int(math.asin(maxL / _ranges[i]) / cell)
        _e = min(len(xxs), i+_n)
        _s = max(0, i-_n)
        vec = []
        for j in range(_s, _e):
            if len(vec) > 4:
                break
            _d = math.sqrt((xxs[i]-xxs[j])**2 + (yys[i]-yys[j])**2)
            if _d > minL and _d < maxL:
                __x, __y = (xxs[j]-xxs[i])/_d, (yys[j]-yys[i])/_d
                for v in vec:
                    __c = abs(__x*v[1] + __y*v[2])
                    print __c
                    if __c < 0.09:
                        plt.plot([xxs[i], xxs[j]], [yys[i], yys[j]], 'b-')
                        plt.plot([xxs[i], xxs[v[0]]], [yys[i], yys[v[0]]], 'k-')
                        break
                vec.append((j, __x, __y))
    plt.show()


if __name__ == '__main__':
    _msg = dict()
    try:
        _f = file('data2.data', 'r')
        while True:
            _msg = cPickle.load(_f)
    except EOFError:
        pass
    shelf(_msg)
