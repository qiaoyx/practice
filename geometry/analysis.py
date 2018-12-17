import string
import math
import pywt
import numpy as np
import scipy as sp
from scipy import interpolate, optimize
from imutils import paths

import matplotlib as mp
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

import pandas as pd

from scipy.fftpack import fft, ifft, fftfreq, fftshift

from sklearn.svm import SVR


def analysis():
    lines = []
    with open("Tracking_180102_085922.txt") as f:
        lines = f.readlines()
    lines.pop(0)

    xcoords = []
    ycoords = []

    def coordinate(line):
        if line is None or len(line) < 7:
            return
        lst = map(string.strip, line.split(","))
        _x = float(lst[5])
        _y = float(lst[6])
        xcoords.append(_x)
        ycoords.append(_y)

    def diff(xs, ys):
        _xs = []
        _ys = []
        map(lambda x, y: _xs.append(y - x), xs[:-1], xs[1:])
        map(lambda x, y: _ys.append(y - x), ys[:-1], ys[1:])
        return _xs, _ys

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

    map(coordinate, lines)
    for i in range(len(xcoords)):
        xcoords[i] = xcoords[i] - xcoords[0]
        ycoords[i] = ycoords[i] - ycoords[0]

    theta, dx, dy = calc_theta(xcoords[0], ycoords[0], 2, 4)
    for i in range(len(xcoords)):
        x2, y2 = transform(dx, dy, theta, xcoords[i], ycoords[i])
        xcoords[i] = x2
        ycoords[i] = y2

    dist = []
    for i in range(len(xcoords)):
        dist.append(math.sqrt(xcoords[i]*xcoords[i] + ycoords[i]*ycoords[i]))
    coords = pd.DataFrame(data=zip(xcoords, ycoords), columns=['x', 'y'])

    xcoords = xcoords[1:]
    ycoords = ycoords[1:]
    dist = dist[1:]

    def difference(list):
        _init = list[:-1]
        _tail = list[1:]
        _ret = []
        for i in range(len(_init)):
            _ret.append(_tail[i] - _init[i])
        return _ret


    def svm_cross_validation(train_x, train_y):
        from sklearn.grid_search import GridSearchCV
        from sklearn.svm import SVC
        model = SVC(kernel='rbf', probability=True)
        param_grid = {'C': [1e-3, 1e-2, 1e-1, 1, 10, 100, 1000],
                      'gamma': [0.001, 0.0001]}
        grid_search = GridSearchCV(model, param_grid, n_jobs=8, verbose=1)
        grid_search.fit(train_x, train_y)
        best_parameters = grid_search.best_estimator_.get_params()
        for para, val in list(best_parameters.items()):
            print(para, val)
        model = SVC(kernel='rbf', C=best_parameters['C'],
                    gamma=best_parameters['gamma'], probability=True)
        model.fit(train_x, train_y)
        return model


    def SVD_FFT(list, thres=10.0):
        _idx = []
        for i in range(len(list)):
            _idx.append(i)
        _fft = fft(list)
        # _frq = fftfreq(len(_fft))
        for i in range(len(_fft)):
            if abs(_fft[i]) < thres:
                _fft[i] = 0
        return ifft(_fft)


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

    cA, cD = pywt.dwt(dist, 'haar')
    cA, cD = pywt.dwt(cA, 'haar')
    ret = SVD_FFT(cA, 10)
    plt.plot(cA,  '-', linewidth=1)

    flag = []
    dl1 = difference(ret)
    _dl1_h = dl1[:-1]
    _dl1_t = dl1[1:]
    flag.append(0)
    flag.append(0)
    for i in range(len(_dl1_h)):
        if _dl1_h[i] * _dl1_t[i] > 0:
            flag.append(0)
        else:
            flag.append(1)

    _z = MyFit(ret)
    plt.plot(_z)

    _ret = map(lambda x, y: abs(y - x), _z, cA)
    # plt.plot(_ret, '--', linewidth=1)

    __x = []
    __y = []
    for i in range(len(flag)):
        if flag[i] == 1:
            __x.append(i)
            __y.append(_ret[i])

    for i in range(len(__x)):
        s = max(0, __x[i] - 20)
        e = min(len(_ret) - 1, __x[i] + 10)

        val = _ret[__x[i]]
        for j in range(s, e):
            if _ret[j] > val:
                val = _ret[j]
                __x[i] = j
        __y[i] = cA[__x[i]]

    plt.plot(__x, __y, 'r+', linewidth=1)

    # ------------ Analysis -----------------------
    az = MyFit(__y, 1)
    plt.plot(__x, az, 'r-')

    plt.grid(True)
    plt.show()

def statistics():
    gt = [0.07308600114029673, -0.008836116523237791, -0.015553804370425084, -0.026211923983415986, -0.022474384337803244, 0.0015012138501724337, 0.005070252650510199, -0.003906006663573436, 0.008761932264359729, 0.00989776093031658, -0.011954242703785312, 0.01163608916094283, 0.005793510497008825, -0.02951968059372234, -0.0191728910785951, -0.018477710742310194, -0.004920206187005505, 0.03248342873430232, 0.018706129161072838, -0.012703580158984096, -0.021227600012707626, -0.012484939514674664, 0.012749636708655032, 0.00596352212765261, 0.014330620316469478, 0.02611033659312234, 0.008675246391952385, -0.019913079884721796, -0.01795208558226058, -0.0016412835077890264, 0.010274046406776094, 0.0019098089114297778]

    ms = [0.13868807199352773, 0.18265669122091488, 0.17834950323353738, 0.04311139645574791, 0.018169080699781404, 0.03434005016718977, -0.08576783745496996, -0.1350310395386547, -0.11452127766548692, -0.016671462903280698, 0.04729771378038805, -0.028591729921554787, -0.17092615406641976, -0.07563954252620064, -0.05145071383850919, 0.03761664884064686, 0.256293456413772, 0.12487507571514378, -0.05338165641020787, 0.03547679794697345, 0.0806050814168131, -0.0001129637827066432, -0.05258692825101363, 0.014895351272837587, 0.1500170512973451, -0.0002655759239615918, -0.08116123089162919, -0.009798517475363866, -0.18103075299900162, -0.20151408686264993, -0.042819017237290424, -0.01347297144209847]

    axis = [i for i in range(len(gt))]
    plt.bar(axis, map(lambda x, y: y - x, ms, gt), facecolor='blue', width=.9)
    plt.bar(axis, ms, facecolor='green', width=.6)
    plt.bar(axis, gt, facecolor='red', width=.2)

    plt.grid(True)
    plt.show()


statistics()