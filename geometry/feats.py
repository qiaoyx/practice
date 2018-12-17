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


def norm(v):  # 计算向量范数
    return np.linalg.norm(v)


def line(msg):
    #  range data preprocess
    _ranges = np.array(_msg['ranges'])
    for i in range(len(_ranges)):  # 设置有效检测距离的范围
        if _ranges[i] < 0.1 or _ranges[i] > 350.0:
            _ranges[i] = 0

    _ddxy = []
    _ration = []
    for i in range(1, len(_ranges)-1):
        _op = np.array([[-1, 1, 0], [0, -1, 1]])
        _xys = np.array(map(
            lambda t: np.array([_ranges[t]*stable[t], _ranges[t]*ctable[t]]),
            xrange(i-1, i+2)))
        _tmp = np.dot(_op, _xys)

        __b = norm(_tmp[0] + _tmp[1])
        if __b == 0.0 or _ranges[i] == 0.0 or __b > 0.20:
            _ration.append(float("inf"))
        else:
            _ration.append((norm(_tmp[0]) + norm(_tmp[1])) / __b)
        _ret = (_tmp[1] - _tmp[0]) * 100.0
        if _ranges[i] == 0.0:
            _ret = np.array([float("inf"), float("inf")])
        _ddxy.append(_ret)

    # _ration = map(lambda r: min(r, 30), _ration)
    _diff = [0] + map(
        lambda v, w, n: abs(np.cross((v+v), (v+w))*(n)),
        _ddxy[:-1], _ddxy[1:], _ration[1:])

    _segs = []
    __thr, __int = 1, 5
    __s, __e = 0, 0
    for i in range(len(_diff)):
        if _ranges[i+1] == 0.0:
            _diff[i] = float("inf")
        if _diff[i] > __thr:
            __e = max(0, i-1)
            if __e - __s > __int:
                _segs.append([__s+2, __e+2])
            __s = i

    plt.figure()
    # p1 = plt.subplot(211)
    # p2 = plt.subplot(212)

    _xs = np.array(map(lambda d, t: d*t, _ranges, stable))
    _ys = np.array(map(lambda d, t: d*t, _ranges, ctable))
    plt.plot(_xs, _ys, 'y.', linewidth=1)
    # plt.plot(_xs[220:260], _ys[220:260], 'b+', linewidth=1)
    # p2.plot([i for i in range(len(_ranges))], _ranges,
    #         # map(lambda c: max(min(c, 10), -10), _ranges),
    #         color=(0., 0., 0.), linestyle=':')

    # _rr = _ranges[200:300]
    # p2.bar([i+200 for i in range(len(_rr))], _rr)
    # p2.plot([i+200 for i in range(1, len(_rr))], np.diff(_rr),
    #         'k-', linewidth=2)

    for i in range(0, len(_segs)):
        plt.plot(_xs[_segs[i][0]:_segs[i][1]],
                 _ys[_segs[i][0]:_segs[i][1]], 'k-', linewidth=2)
        # p2.plot([j for j in range(_segs[i][0], _segs[i][1])],
        #         _ranges[_segs[i][0]:_segs[i][1]], 'k-')

        __s, __e = _segs[i][0], _segs[i][1]
        _f = np.array([_xs[__s], _ys[__s]]),
        _e = np.array([_xs[__e-1], _ys[__e-1]])
        _max, _idx = 0, __s
        _c, _a, _b = norm(_f - _e), 0, 0
        for j in range(__s+1, __e-2):
            _p = np.array([_xs[j], _ys[j]])
            _a, _b = norm(_p - _f), norm(_p - _e)
            _l = (_a + _b + _c) / 2.0
            _d = 2.0 * math.sqrt(_l*(_l-_a)*(_l-_b)*(_l-_c)) / _c
            if _d > _max:
                _max, _idx = _d, j

        if _max > 0.04:  # corner case
            plt.plot(_xs[_idx], _ys[_idx], 'r*')
        else:  # error case, look like a line.
            plt.plot(_xs[_idx], _ys[_idx], 'y.')

    # for i in range(0, len(_segs)):
        # __s, __e = _segs[i][0], _segs[i][1]
        # cnt = (np.array([_xs[__s:__e], _ys[__s:__e]])*1000).astype(
        #     np.int32).T.reshape(-1, 1, 2)
        # area = cv2.minAreaRect(cnt)
        # hull = cv2.convexHull(cnt, returnPoints=False)
        # defects = cv2.convexityDefects(cnt, hull)

        # _idx, _max = 0, 0
        # for i in range(defects.shape[0]):
        #     s, e, f, d = defects[i, 0]
        #     if d > _max:
        #         _idx, _max = f, d
        #     far = tuple(cnt[f][0])
        #     p1.plot(far[0]/1000.0, far[1]/1000.0, 'r.')
        # far = tuple(cnt[_idx][0])
        # if far > 10000.0:
        #     p1.plot(far[0]/1000.0, far[1]/1000.0, 'b*')

        # hull = cv2.convexHull(cnt)
        # box = np.array(cv2.boxPoints(area)) / 1000.0
        # ax, ay = list(box.T[0]), list(box.T[1])
        # p1.plot(ax + [ax[0]], ay + [ay[0]], 'k-')

        # ax, ay = [], []
        # for i in range(len(hull)):
        #     ax.append(hull[i][0][0] / 1000.0)
        #     ay.append(hull[i][0][1] / 1000.0)
        # p1.plot(ax + [ax[0]], ay + [ay[0]], 'b-')

    plt.show()


if __name__ == '__main__':
    # myFile =file('data.data','a') # cPickle.dump(msg,myFile) # myFile.close()
    _msg = dict()
    try:
        _f = file('data2.data', 'r')
        while True:
            _msg = cPickle.load(_f)
    except EOFError:
        pass
    line(_msg)
