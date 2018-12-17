import cv2
import math
import string
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class bezier5:
    """ 贝塞尔曲线的系数矩阵 """
    Mk = np.array([[1.0,    0.0,   0.0,   0.0,  0.0, 0.0],
                   [-5.0,   5.0,   0.0,   0.0,  0.0, 0.0],
                   [10.0, -20.0,  10.0,   0.0,  0.0, 0.0],
                   [-10.0, 30.0, -30.0,  10.0,  0.0, 0.0],
                   [5.0,  -20.0,  30.0, -20.0,  5.0, 0.0],
                   [-1.0,   5.0, -10.0,  10.0, -5.0, 1.0]])
    """ 反算系数矩阵 """
    Hk = np.array([[1.0,   0.0,    0.0,    0.0,    0.0,   0.0],
                   [1.0, 1/5.0,    0.0,    0.0,    0.0,   0.0],
                   [1.0, 2/5.0, 1/20.0,    0.0,    0.0,   0.0],
                   [0.0,   0.0,    0.0, 1/20.0, -2/5.0,   1.0],
                   [0.0,   0.0,    0.0,    0.0, -1/5.0,   1.0],
                   [0.0,   0.0,    0.0,    0.0,    0.0,   1.0]])
    dist_thres = 0.01  # 最小距离阈值
    ratio = 3.0  # 切向量缩放系数

    carWr = 0.633 / 2.0  # 半车宽
    carHr = 0.633 / 2.0  # 半车长

    def __init__(self, xs, ys):  # 参数为waypoints坐标
        xxs, yys = [], []
        xxs.append(xs[0])
        yys.append(ys[0])
        for i in xrange(1, len(xs)):  # 距离太近的点，忽略
            d = math.sqrt((xs[i]-xxs[-1])**2 + (ys[i]-yys[-1])**2)
            if d > self.dist_thres:
                xxs.append(xs[i])
                yys.append(ys[i])
        if (len(xxs) != len(yys)) or (len(xxs) < 2):  # 直线
            return
        self.waypoints = np.array(zip(xxs, yys))
        if len(xxs) == 2:
            return

        delt = np.diff(self.waypoints.T).T  # transit segments to vectors.
        T = np.diff(delt.T).T  # calculate norm vector.
        self.T = np.array(map(lambda p: [p[1], -p[0]], T))  # calculate tangent vector
        for i in xrange(self.T.shape[0]):
            # calc the rotation direction of tangent vector
            sign = np.cross(delt[i], delt[i+1])
            if sign < 0:
                self.T[i] = -self.T[i]

            sign2 = np.cross(self.T[i], delt[i+1])
            sign3 = np.cross(delt[i], self.T[i])
            if sign * sign2 <= 0:
                self.T[i] = (delt[i] + delt[i+1]) / 2.0
            if sign * sign3 <= 0:
                self.T[i] = (delt[i] + delt[i+1]) / 2.0

        self.T = np.insert(self.T, 0, values=delt[0], axis=0)
        self.T = np.append(self.T, np.array([delt[-1]]), axis=0)
        for i in xrange(0, delt.shape[0] - 1):
            n1 = np.linalg.norm(self.T[i+1])
            n2 = (math.sqrt(delt[i][0]**2 + delt[i][1]**2) +
                  math.sqrt(delt[i+1][0]**2 + delt[i+1][1]**2)) / self.ratio
            if n1 > 0.0001:
                self.T[i+1] = self.T[i+1] * n2 / n1  # normalization
            else:
                self.T[i+1] = delt[i+1] / self.ratio

        # transit tangent to vectors.
        A = np.diff(self.T.T).T
        self.A = np.array(map(lambda p: [p[1], -p[0]], A))
        for i in xrange(self.A.shape[0]):
            sign = np.cross(self.T[i], self.T[i+1])
            if sign < 0:
                self.A[i] = -self.A[i]
            sign2 = np.cross(self.A[i], self.T[i+1])
            sign3 = np.cross(self.T[i], self.A[i])
            if sign * sign2 <= 0:
                # self.A[i] = self.T[i+1]
                self.A[i] = (self.T[i] + self.T[i+1]) / 2.0
            if sign * sign3 <= 0:
                # self.A[i] = self.T[i]
                self.A[i] = (self.T[i] + self.T[i+1]) / 2.0

        self.A = np.insert(self.A, 0, values=self.T[0], axis=0)
        for i in xrange(0, self.A.shape[0] - 1):
            n1 = np.linalg.norm(self.A[i+1])
            n2 = (math.sqrt(self.T[i][0]**2 + self.T[i][1]**2) +
                  math.sqrt(self.T[i+1][0]**2 + self.T[i+1][1]**2))/self.ratio
            if n1 > 0.00001:
                self.A[i+1] = self.A[i+1] * n2 / n1
            else:
                self.A[i+1] = self.T[i+1] / self.ratio

    def tv(self, t):  # parameter t vector
        return np.array([t ** i for i in xrange(6)])

    def dt(self, t):  # differential of parameter t vector
        return np.array([0, 1.0, t/2.0, (t**2)/3.0, (t**3)/4.0, (t**4)/5.0])

    def d2t(self, t):  # 2 order differential of parameter t vector
        return np.array([0, 0, 0.5, t/6.0, (t**2)/12.0, (t**3)/20.0])

    def curve2(self):  # obsolete
        ts = np.array([i / 100.0 for i in xrange(101)])
        ps0 = self.waypoints[:-1]
        ps5 = self.waypoints[1:]
        ps1 = ps0 + self.T[:-1] / 5.0
        ps4 = ps5 - self.T[1:] / 5.0
        ps2 = map(lambda a, p1, p0: a/20.0 + 2*p1 - p0, self.A[:-1], ps1, ps0)
        ps3 = map(lambda a, p4, p5: a/20.0 + 2*p4 - p5, self.A[1:], ps4, ps5)

        coords, curvature = [], []
        for i in xrange(len(ps0) - 1):
            cs = np.array([ps2[i], ps3[i], ps4[i], ps5[i], ps0[i+1], ps1[i+1]])
            coords.append(
                map(lambda t: np.dot(np.dot(self.tv(t), self.Mk), cs), ts))
            dts = map(lambda t: np.dot(np.dot(self.dt(t), self.Mk), cs), ts)
            d2ts = map(lambda t: np.dot(np.dot(self.d2t(t), self.Mk), cs), ts)
            a = map(lambda x, y: np.cross(x, y), dts, d2ts)
            b = map(lambda dd: np.linalg.norm(dd) ** 3, dts)
            curvature.append(map(lambda l, m: l / m, a, b))
        return coords, curvature

    def curve(self):
        coords, curvature, car = [], [], []  # 坐标, 曲率, 车身轮廓
        ts = np.array([i / 100.0 for i in xrange(101)])  # t参数化取值
        if self.waypoints.shape[0] == 2:  # line case
            _c = np.array(map(lambda t: np.array([1-t, t]), ts))
            coords.append(map(lambda t: np.dot(self.waypoints.T, t.T), _c))
            curvature.append(map(lambda t: 0, _c))
            return coords, curvature

        ps0 = self.waypoints[:-1]
        ps5 = self.waypoints[1:]
        ps1 = ps0 + self.T[:-1] / 5.0
        ps4 = ps5 - self.T[1:] / 5.0
        ps2 = map(lambda a, p1, p0: a/20.0 + 2*p1 - p0, self.A[:-1], ps1, ps0)
        ps3 = map(lambda a, p4, p5: a/20.0 + 2*p4 - p5, self.A[1:], ps4, ps5)

        for i in xrange(len(ps0)):
            cs = np.array([ps0[i], ps1[i], ps2[i], ps3[i], ps4[i], ps5[i]])
            _cds = map(lambda t: np.dot(np.dot(self.tv(t), self.Mk), cs), ts)
            dts = map(lambda t: np.dot(np.dot(self.dt(t), self.Mk), cs), ts)
            _dts = map(lambda d: d / np.linalg.norm(d), dts)
            _nrm = map(lambda d: np.array([-d[1], d[0]]), _dts)
            _car = []
            for k in range(len(_cds)):
                a, b = _nrm[k][0]*self.carWr, _nrm[k][1]*self.carWr
                c, d = _dts[k][0]*self.carHr, _dts[k][1]*self.carHr
                x, y = _cds[k][0] + a + c, _cds[k][1] + b + d
                _car = _car + [[x, y]]  # plt.plot(x, y, 'y.')
                x, y = _cds[k][0] - a - c, _cds[k][1] - b - d
                _car = _car + [[x, y]]  # plt.plot(x, y, 'y.')
                x, y = _cds[k][0] + a - c, _cds[k][1] + b - d
                _car = _car + [[x, y]]  # plt.plot(x, y, 'y.')
                x, y = _cds[k][0] - a + c, _cds[k][1] - b + d
                _car = _car + [[x, y]]  # plt.plot(x, y, 'y.')
            d2ts = map(lambda t: np.dot(np.dot(self.d2t(t), self.Mk), cs), ts)
            a = map(lambda x, y: np.cross(x, y), dts, d2ts)
            b = map(lambda dd: np.linalg.norm(dd) ** 3, dts)
            car = car + _car
            coords.append(_cds)
            curvature.append(map(lambda l, m: l / m, a, b))
        return coords, curvature, car


def parseData(file):
    lines, xs, ys = [], [], []
    with open(file) as f:
        lines = f.readlines()
    for i in xrange(len(lines)):
        vs = lines[i].split(',')
        if len(vs) > 3:
            xs.append(float(string.strip(vs[1])))
            ys.append(float(string.strip(vs[2])))
    return xs, ys


def test():
    file = "/home/qiaoyx/workspace/Laser_Project/map/factory/0611/Mainload/A+O+Q.txt"
    xs, ys = parseData(file)

    rc = bezier5(xs, ys)
    coords, curvature, car = rc.curve()
    x = map(lambda p: p[0], rc.waypoints)
    y = map(lambda p: p[1], rc.waypoints)

    flag = True  # plot with 2D or 3D

    # 绘制车体扫过的区域
    car = np.array(car)
    xcar = car.T[0]
    ycar = car.T[1]
    plt.plot(xcar, ycar, color='gray', linestyle='--')

    if flag:
        plt.plot(x, y, linestyle='-', color='black', linewidth=16)
        plt.plot(x, y, linestyle='-', color='gray',  linewidth=1)
        for i in xrange(len(coords)):
            x = map(lambda p: p[0], coords[i])
            y = map(lambda p: p[1], coords[i])
            plt.plot(x, y, color='cyan', linewidth=1)
    else:
        fig = plt.figure()
        ax = Axes3D(fig)
        plt.plot(x, y, linestyle='-', color='black', linewidth=16)
        plt.plot(x, y, linestyle='-', color='gray',  linewidth=1)
        for i in xrange(len(coords)):
            x = map(lambda p: p[0], coords[i])
            y = map(lambda p: p[1], coords[i])
            c = map(lambda p: p,    curvature[i])
            ax.scatter(x, y, c, c='green', s=5)
            plt.plot(x, y, 'b-', linewidth=1)
    plt.show()


test()
