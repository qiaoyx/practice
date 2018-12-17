import string
import math
import json
import copy
import pywt
import numpy as np
import scipy as sp
import pandas as pd
from scipy import interpolate, optimize
from imutils import paths

from sklearn.linear_model import Lasso
from sklearn.metrics import r2_score

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.fftpack import fft, ifft, fftfreq, fftshift


class MyDataSet:
    dataFile = ""
    B = np.array([1, 2, 4, 2, 1])
    K = 10

    def __init__(self, data_file=""):
        self.dataFile = data_file

    def parseData(self):
        lines = []
        with open(self.dataFile) as f:
            lines = f.readlines()
        lines.pop(0)

        vectors = map(lambda l: map(
            lambda v: float(string.strip(v)), l.split(',')[2:]), lines)
        vectors = filter(lambda l: len(l) > 5, vectors)
        cross = [0] + \
                map(lambda x, y: np.cross([x[3], -x[4]], [y[3], -y[4]]),
                vectors[:-1], vectors[1:])

        factor = 1
        self.df = pd.DataFrame(
            data=map(lambda l,c: (l[3], -l[4],
                             math.sqrt(l[3]*l[3] + l[4]*l[4] + factor*c*c),
                             l[5], l[0], l[1], c, 0), vectors, cross),
            columns=['x', 'y', 'd', 'z', 'hzangle', 'vangle', 'cross', 'f'])

        # self.df = self.df[self.df.d > 138]
        # self.df = self.df[self.df.d < 144]
        # self.df = self.df[self.df.d < 139.2]

    def parseData2(self):
        lines = []
        with open(self.dataFile) as f:
            lines = f.readlines()
        lines.pop(0)

        vectors = map(lambda l: map(
            lambda v: float(string.strip(v)), l.split(' ')), lines)
        vectors = filter(lambda l: len(l) > 1, vectors)
        cross = [0] + map(lambda a, b: np.cross(a[:2], b[:2]),
                          vectors[:-1], vectors[1:])
        _data = map(lambda l, c:
                    (l[0], l[1], math.sqrt(l[0]*l[0] + l[1]*l[1] + c*c),
                     0, 0, 0, c, 0), vectors, cross)

        self.df = pd.DataFrame(
            data=_data,
            columns=['x', 'y', 'd', 'z', 'hzangle', 'vangle', 'cross', 'f'])

    def calc_transform(self, x1, y1, x2, y2):
        if x1 == 0 or y1 == 0 or x2 == 0 or y2 == 0:
            return 0, 0, 0

        theta1 = math.acos(x1 / math.sqrt(x1 * x1 + y1 * y1))
        theta2 = math.acos(x2 / math.sqrt(x2 * x2 + y2 * y2))
        dtheta = theta1 - theta2
        dx = x2 - x1*math.cos(dtheta) + y1*math.sin(dtheta)
        dy = y2 - x1*math.sin(dtheta) - y1*math.cos(dtheta)

        return dtheta, dx, dy

    def wavelet(self, d, t=1):
        cA = d
        cD = d
        for i in range(t):
            cA, cD = pywt.dwt(cA, 'haar')

        for i in range(len(cD)):
            if 0 == cD[i]:
                self.df['f'][(2 ** t)*i] = 1

        self.fix_pos = self.df[self.df.f == 1]
        ds = self.fix_pos['x'].diff()
        for i in range(len(ds)):
            v = ds.values[i]
            thres = 1e-3
            if v > thres:
                self.fix_pos['f'][ds.index[i-1]] = 1
            else:
                if v < -thres:
                    self.fix_pos['f'][ds.index[i-1]] = 2
                else:
                    self.fix_pos['f'][ds.index[i-1]] = 0
        self.justify()

    def justify(self):
        a = self.df[self.df.f == 1]
        b = self.df[self.df.f == 2]
        # _x = a['x'].median() - b['x'].median()
        # _y = a['y'].median() - b['y'].median()
        # _x = b['x'].median() - 85.5
        # _y = b['y'].median() - 89.0
        # theta = math.atan(_y/_x) / 2
        # theta = math.pi / 5.5
        theta = -math.pi / 7.4
        self.transform(theta)

    def transform(self, theta):
        C = math.cos(theta)
        S = math.sin(theta)
        for j in range(len(self.df)):
            x = self.df.iloc[j, 0]
            y = self.df.iloc[j, 1]
            self.df.iat[j, 0] = x * C - y * S
            self.df.iat[j, 1] = x * S + y * C

#       dx = -self.fix_pos[self.fix_pos.f != 0]['x'].median()
#       dy = -self.fix_pos[self.fix_pos.f != 0]['y'].median()
        dx = -self.df[self.df.f != 0]['x'].median()
        dy = -self.df[self.df.f != 0]['y'].median()

        for j in range(self.df['x'].size):
            x = self.df.iloc[j, 0]
            y = self.df.iloc[j, 1]
            self.df.iat[j, 0] = x + dx
            self.df.iat[j, 1] = y + dy

    def svd_fft(self, d, thres=10.0):
        _fft = fft(d)
        for i in range(len(_fft)):
            if _fft[i] <= thres:
                _fft[i] = 0
        return ifft(_fft)

    def fit(self, d, order=1):
        def polyfunc(coef, x):
            ret = 0
            L = len(coef) - 1
            for i in range(len(coef)):
                ret += coef[L - i] * math.pow(x, i)
            return ret

        _x = []
        for i in range(len(d)):
            _x.append(i)
        _y = np.polyfit(_x, d, order)
        _z = map(lambda x: polyfunc(_y, x), _x)
        return _z

    def statistics(self):
        def autolabel(rects):
            """
            Attach a text label above each bar displaying its height
            """
            for rect in rects:
                height = rect.get_height()
                plt.text(rect.get_x() + rect.get_width()/2.,
                         .6*height, '%.2f' % float(height),
                         ha='center', va='bottom', rotation=90, size=12)

        for k in [1, 2]:
            data = self.df[self.df.f == k]

            deltx = data['x'].values.tolist()[1:]
            delty = data['y'].values.tolist()[1:]

            xmean = data['x'].median()
            ymean = data['y'].median()
            # xmean = data['x'].iat[2]
            # ymean = data['y'].iat[2]
            # deltx = map(lambda x: math.sqrt((x-xmean)**2)*100.0, deltx)
            # delty = map(lambda y: math.sqrt((y-ymean)**2)*100.0, delty)
            deltx = map(lambda x: (x-xmean)*100.0, deltx)
            delty = map(lambda y: (y-ymean)*100.0, delty)

            idx = []
            for i in range(len(deltx)):
                idx.append(i)

            r = plt.subplot(310+k)
            if k == 1:
                r1 = r.bar(idx, deltx, .45, facecolor='red', label="X Bias")
                autolabel(r1)
            else:
                r1 = r.bar(idx, deltx, .45, facecolor='blue', label="X Bias")
                autolabel(r1)

            idx = [i+0.45 for i in idx]
            if k == 1:
                r2 = r.bar(idx, delty, .45, facecolor='blue', label="Y Bias")
                autolabel(r2)
            else:
                r2 = r.bar(idx, delty, .45, facecolor='red', label="Y Bias")
                autolabel(r2)

            # r.plot([2 for i in idx], 'y-')
            # r.plot([1 for i in idx], 'g-')
            # r.axis([-0.5, len(idx), 0, max(delty + deltx) + 0.5])
            if k == 1:
                plt.xlabel(r"Survey Times of Point Red")
            else:
                plt.xlabel(r"Survey Times of Point Blue")
            plt.ylabel(r"Standard Deviation Of Coords Bias (cm)")
            plt.legend(loc='upper right')
            plt.grid(True)

        plt.subplot(313)
        p1 = self.df[self.df.f == 1]
        p2 = self.df[self.df.f == 2]
        plt.plot(self.df['x'], self.df['y'], 'k--', linewidth=1)
        plt.plot(p1['x'], p1['y'], 'r*')
        plt.plot(p2['x'], p2['y'], 'b*')
        plt.xlabel(r"X Coordinates (m)")
        plt.ylabel(r"Y Coordinates (m)")
        plt.grid(True)
        plt.show()

    def process2(self):
        if self.dataFile.startswith('Tracking'):
            self.parseData()
        else:
            self.parseData2()

        D = [0, 0] + self.df['cross'].values.tolist() + [0, 0]
        cA, cD = pywt.dwt(D, 'db2')
        for i in range(len(cD)):
            if abs(cD[i]) < 1e-2:
                cD[i] = 0.0
        D = pywt.idwt(cA, cD, 'db2')

        flags = []
        for i in range(2, len(D)-2):
            A = np.array(D[i-2:i+3])
            flags.append(abs(np.dot(self.B, A.T) / self.K - D[i]) < 2e-3)

        l = len(flags)
        for i in range(l):
            h = max(0, i-20)
            t = min(i+20, l-1)
            sub = filter(lambda x: x == True, flags[h:t])
            if not (len(sub) > 2 and flags[i]):
                flags[i] = False
        self.df['f'] = flags

        kvs = {}
        DS = self.df[self.df.f == True]['d']
        for i in range(DS.size):
            k = int(DS.iat[i] * 1)
            if k in kvs:
                kvs[k].append(DS.index[i])
            else:
                kvs[k] = [DS.index[i]]

        # self.df['f'] = flags
        plt.plot(self.df['d'], '.',  color='lightgray')
        # plt.plot(self.df[self.df.f == True]['d'], '*', color='red')
        # plt.grid(True)
        # plt.show()

        vs = kvs.values()
        for i in range(len(vs)):
            if len(vs[i]) > 3:
                plt.plot(vs[i], map(lambda x: DS[x], vs[i]), '+')
        plt.show()

    def process(self):
        if self.dataFile.startswith('Tracking'):
            self.parseData()
        else:
            self.parseData2()
        """ use lie group [x y [x,y]] map next. """

        Zeros = self.df[abs(self.df.cross) < 1e-5]
        Zero_D = Zeros['d']
        dm = Zero_D.mean()

        flag = (Zero_D[0] - dm) > 0
        # if (flag):
        #     self.df['f'][Zero_D.index[0]] = 1
        # else:
        #     self.df['f'][Zero_D.index[0]] = 2

        distance = Zero_D.values[1:]
        xs = Zeros['x'].values[1:]
        ys = Zeros['y'].values[1:]
        for i in range(len(distance)):
            _f = ((distance[i] - dm) > 0)
            if (_f != flag):
                flag = _f
                if (flag):
                    self.df['f'][Zero_D.index[i+1]] = 1
                #     if not (xs[i] > 96.26 and xs[i] < 96.57 and \
                #             ys[i] < -96.32 and ys[i] > -96.62):
                #         self.df['f'][Zero_D.index[i+1]] = 1
                #     else:
                #         flag = not flag
                else:
                    self.df['f'][Zero_D.index[i+1]] = 2

        self.justify()
        self.statistics()

    def check(self):
        anchor = self.df[abs(self.df.cross) < 1e-5]['d']
        plt.plot(self.df['d'], '.',  color='lightgray')
        plt.plot(self.df[self.df.f != 0]['d'], '-', color='black', linewidth=1)
        plt.plot(anchor, '.', color='red')
        plt.plot(self.df[self.df.f == 1]['d'], '*', color='red')
        plt.plot(self.df[self.df.f == 2]['d'], '*', color='blue')
        plt.grid(True)
        plt.show()

        fig = plt.figure()
        ax = Axes3D(fig)
        xs = self.df['x'].values
        ys = self.df['y'].values
        zs = self.df['cross'].values
        # Zeros = self.df[abs(self.df.cross) < 1e-5]
        Zeros = self.df[self.df.f != 0]
        ax.scatter(xs, ys, zs, '--+', s=5)
        ax.scatter(Zeros['x'], Zeros['y'], Zeros['cross'], c='red', s=50)
#        ax.plot_surface(xs, ys, zs, rstride=1,cstride=1,cmap=plt.cm.jet)
        plt.show()


md = MyDataSet("Tracking_180130_101602.txt")
# md = MyDataSet("amcl_pose.txt")
md.process()
md.check()
