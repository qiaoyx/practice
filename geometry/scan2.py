import math
import numpy as np
import matplotlib.pyplot as plt


def hat(vec):  # vector to it's correspondence skew matrix
    return np.array([[0, -vec[2], vec[1]],
                     [vec[2], 0, -vec[0]],
                     [-vec[1], vec[0], 0]])


def scale(vec):  # [x, y, z] => [x/z, y/z, 1.0]
    if vec[2] == 0:
        return
    else:
        vec[0] = 1.0 * vec[0] / vec[2]
        vec[1] = 1.0 * vec[1] / vec[2]
        vec[2] = 1.0


cell = 1.0 * math.pi / 180.0
angle_table = map(lambda i: float(i) * cell, xrange(360))  # table of angle
ctable = map(lambda t: math.cos(t), angle_table)  # table of sin function
stable = map(lambda t: math.sin(t), angle_table)  # table of cos function


def line(p1, p2):  # two different points determine one line exactly.
    a = np.array([p1[0], p1[1], 1])
    b = np.array([p2[0], p2[1], 1])
    c = np.dot(hat(a), b)
    scale(c)
    return c


def drawLines(lines, cs='b-'):
    for l in lines:
        _x, _y = [], []
        for i in xrange(-50, 75):
            y = -(l[0]*i+l[2]) / l[1]
            if y > -50 and y < 75:
                _x.append(i)
                _y.append(y)
        plt.plot(_x, _y, cs)


# select two points of one scan, calculate a line through them, then calculate
# the norm line of it which through the middle of the two points.
def bin(norms, lines, xs, ys, low, high):
    l, h = low, high
    if l >= h:
        return
    m = (high + low) / 2
    _l = line((xs[l], ys[l]), (xs[h], ys[h]))
    _x, _y = (xs[l] + xs[h]) / 2.0, (ys[l] + ys[h]) / 2.0
    _c = _l[1]*_x - _l[0]*_y
    lines.append(_l)
    norms.append(np.array([-_l[1], _l[0], _c]))

    if l != m:
        _l = line((xs[l], ys[l]), (xs[m], ys[m]))
        _x, _y = (xs[l] + xs[m]) / 2.0, (ys[l] + ys[m]) / 2.0
        _c = _l[1]*_x - _l[0]*_y
        lines.append(_l)
        norms.append(np.array([-_l[1], _l[0], _c]))

    _l = line((xs[m], ys[m]), (xs[h], ys[h]))
    _x, _y = (xs[m] + xs[h]) / 2.0, (ys[m] + ys[h]) / 2.0
    _c = _l[1]*_x - _l[0]*_y
    lines.append(_l)
    norms.append(np.array([-_l[1], _l[0], _c]))

    bin(norms, lines, xs, ys, l+1, m-1)
    bin(norms, lines, xs, ys, m+1, h-1)


def test():
    r = 25.0
    xs = [(10+r*c) for c in ctable]
    ys = [(20+r*s) for s in stable]

    xxs = [xs[10*i] for i in xrange(len(xs)/20)]
    yys = [ys[10*i] for i in xrange(len(ys)/20)]

    # 加入随机误差
    xxs = xxs + np.random.normal(0, 3, len(xxs))
    yys = yys + np.random.normal(0, 3, len(yys))

    lines, norms = [], []

    bin(norms, lines, xxs, yys, 0, len(xxs)-1)
    drawLines(lines)
    drawLines(norms, 'y--')

    _xs, _ys = [], []
    for i in range(len(norms)-1):
        # two different lines intersected in one point.
        c = np.cross(norms[i], norms[i+1])
        scale(c)
        _xs.append(c[0])
        _ys.append(c[1])

    centerX, centerY = np.array(_xs).mean(), np.array(_ys).mean()
    _tmp = map(lambda x, y: abs(x-centerX) + abs(y-centerY), _xs, _ys)
    _cx, _cy = [], []
    for i in range(len(_tmp)):
        if _tmp[i] < r:
            _cx.append(_xs[i])
            _cy.append(_ys[i])

    plt.plot([10], [20], 'ko')
    plt.plot(_cx, _cy, 'yo')
    plt.plot(np.array(_cx).mean(), np.array(_cy).mean(), 'k*')

    plt.plot(xs, ys, 'k-')
    plt.plot(xxs, yys, 'ro')

    plt.show()


test()
