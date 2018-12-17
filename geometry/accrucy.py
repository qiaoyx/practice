# coding=utf-8
import math
import string
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class MyDataSet:
    def __init__(self, data_file=""):
        self.dataFile = data_file

    def parseData(self):  # 数据读取与解析
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
            data=map(lambda l, c: (l[3], -l[4],
                math.sqrt(l[3]*l[3] + l[4]*l[4] + factor*c*c),
                l[5], l[0], l[1], c, 0), vectors, cross),
            columns=['x', 'y', 'd', 'z', 'hzangle', 'vangle', 'cross', 'f'])

    def parseData2(self):  # 数据读取与解析
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

    def justify(self):  # 旋转地图theta角度
        self.transform(-math.pi / 7.4)

    def transform(self, theta):  # 旋转地图点坐标并居中，方便显示
        C, S = math.cos(theta), math.sin(theta)
        for j in range(len(self.df)):  # 旋转坐标点
            x, y = self.df.iloc[j, 0], self.df.iloc[j, 1]
            self.df.iat[j, 0] = x * C - y * S
            self.df.iat[j, 1] = x * S + y * C

        # 计算坐标中心点
        dx = -self.df[self.df.f != 0]['x'].median()
        dy = -self.df[self.df.f != 0]['y'].median()
        # 坐标居中处理
        for j in range(self.df['x'].size):
            x, y = self.df.iloc[j, 0], self.df.iloc[j, 1]
            self.df.iat[j, 0] = x + dx
            self.df.iat[j, 1] = y + dy

    def statistics(self):
        def autolabel(rects):  # 绘图时显示的label信息
            """
            Attach a text label above each bar displaying its height
            """
            for rect in rects:
                height = rect.get_height()
                plt.text(rect.get_x() + rect.get_width()/2.,
                         .6*height, '%.2f' % float(height),
                         ha='center', va='bottom', rotation=90, size=12)

        for k in [1, 2]:  # 分别处理第一停靠点和第二停靠点
            data = self.df[self.df.f == k]  # 筛选停靠点
            deltx = data['x'].values.tolist()[1:]
            delty = data['y'].values.tolist()[1:]

            xmean, ymean = data['x'].median(), data['y'].median()
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

    def process(self):
        if self.dataFile.startswith('Tracking'):
            self.parseData()
        else:
            self.parseData2()

        # 设置一个线性分类器，用于分离两个停靠点
        Zeros = self.df[abs(self.df.cross) < 1e-5]  # 阈值筛选，小于该值认为是静止停靠的
        Zero_D = Zeros['d']  # 筛选出停靠点距离原点的距离(坐标已做居中处理，停靠点在不同象限)
        dm = Zero_D.mean()  # 计算距离均值

        flag = (Zero_D[0] - dm) > 0  # 以距离均值为分类器条件，进行数据分离
        distance = Zero_D.values[1:]
        for i in range(len(distance)):  # 二分类过程, flag用于标记属于哪一类
            _f = ((distance[i] - dm) > 0)
            if (_f != flag):
                flag = _f
                if (flag):
                    self.df['f'][Zero_D.index[i+1]] = 1  # 第一停靠点
                else:
                    self.df['f'][Zero_D.index[i+1]] = 2  # 第二停靠点

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

        # fig = plt.figure()
        # ax = Axes3D(fig)
        # xs = self.df['x'].values
        # ys = self.df['y'].values
        # zs = self.df['cross'].values
        # Zeros = self.df[self.df.f != 0]
        # ax.scatter(xs, ys, zs, '--+', s=5)
        # ax.scatter(Zeros['x'], Zeros['y'], Zeros['cross'], c='red', s=50)
        plt.show()


md = MyDataSet("Tracking_180130_101602.txt")
md.process()
md.check()
