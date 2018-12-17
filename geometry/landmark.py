import rospy
from sklearn import svm
from tf import TransformListener, TransformBroadcaster, transformations

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
# from math import fabs, cos, sin, pi, floor, atan2
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Mark:  # 以线段表示的LandMark类
    thres = 0.2

    def __init__(self, q=np.array([0.0, 0.0]), tq=np.array([0.0, 0.0])):
        """
        参数为4分之一点和4分之3点, 以两点方式表示一个线段，即Mark对象
        """

        self.n = 1.0  # 表示累加的次数，用于计算均值
        self.quarter = q  # 4分之一点累加值
        self.three_quarter = tq  # 4分之3点累加值
        self.central = (q + tq) / 2.0

        # 该Mark的全局信息由方向角度和中心点位置来表示
        self.rt = np.array([100.0, 0.0, 0.0])  # [theta, dx, dy]

    def mean(self, mark):
        """
        计算一个Mark对象的中心点, 并以Mark的形式返回
        """
        _m = Mark(self.quarter/self.n, self.three_quarter/self.n)

        _v0 = mark.three_quarter - mark.central  # to vector
        _v1 = _m.three_quarter - _m.central  # to vector
        if (np.linalg.norm(_v0) * np.linalg.norm(_v1)) == 0:
            return _m

        # 计算两个向量间的夹角
        _cos = np.dot(_v0, _v1.T) / (np.linalg.norm(_v0) * np.linalg.norm(_v1))
        _theta = math.acos(_cos)

        _sgn = np.cross(_v1, _v0)  # _v1 -> _v0 : _v0 - _v1
        if _sgn >= 0:  # theta >= 0
            pass
        else:  # theta < 0
            _theta = -_theta
        _offset = mark.central - _m.central

        _m.rt[0] = _theta
        _m.rt[1] = _offset[0]
        _m.rt[2] = _offset[1]

        return _m

    def equals(self, c=np.array([0.0, 0.0])):  # 判断两个Mark中心点位置是否相同
        delt = self.central - c
        dist = np.linalg.norm(delt)
        return dist < self.thres

    def equals2(self, m):
        d1 = np.linalg.norm(self.three_quarter - self.quarter)
        d2 = np.linalg.norm(m.three_quarter - m.quarter)
        if abs(d1-d2) < self.thres and self.equals(m.central):
            return True
        return False

    def update(self, m):
        """
        如果判定m与当前Mark相同，则累加。否则，返回False
        """
        if not self.equals(m.central):
            return False
        self.n = self.n + 1
        self.quarter = self.quarter + m.quarter
        self.three_quarter = self.three_quarter + m.three_quarter
        self.central = self.mean(Mark()).central
        return True


class reflector:  # 反光板类
    angle_cell = 0.25 * math.pi / 180.0
    residual_thr = 0.002  # threshold of residual error (m).
    reflector_width = 0.45  # width of the reflector (m).
    width_error = 0.02  # width error accept range (m).

    # how many lidar radials interval between two reflectors.
    segment_interval = 2

    # True: calc pose by reflector central.
    # False: calc reflector central by pose.
    calc_orientation = True

    def __init__(self, msg):
        _len = len(msg.ranges)
        self.msg = msg
        self.cells = map(lambda i: -(i-_len/2.0)*self.angle_cell, xrange(_len))
        for i in xrange(len(self.msg.ranges)):
            if math.isinf(self.msg.ranges[i]) or self.msg.ranges[i] > 40.0:
                self.msg.ranges[i] = 0.0
            if math.isinf(self.msg.intensities[i]):
                self.msg.intensities[i] = 0.0
        self.segments = []
        self.coords = []
        self.centrals = []
        self.quarters = []
        self.three_quarters = []
        self.lens = []
        self.poses = []

    def train_svm(self, data):
        pass

    def recognise(self):  # 识别反光条
        _last, _segs, _d = 0, [], self.msg.ranges[0]
        for i in xrange(len(self.msg.ranges)):
            __d = self.msg.ranges[i]
            if __d >= 10.0:  # 只处理精度范围内的点
                __d = 0
            else:  # 被选中后，将距离转换为与强度相关的阈值，用于区分反光板
                __d = math.sqrt(__d / 10.0)
            # 反光强度与距离成反比，同反射强度成正比
            _ret = __d * self.msg.intensities[i]
            if _ret > 400.0:  # 400.0为经验阈值
                _interval = i - _last - 1  # 记录相邻的检测到反光板的激光束数量
                _last = i  # 上一个检测到的激光束下标
                _delt = abs(self.msg.ranges[i] - _d)  # 相邻激光束间的距离差
                _d = self.msg.ranges[i]
                _segs.append((_interval, i, _delt))

        _start, _end, _len = [], [], len(_segs)
        for i in xrange(_len):
            if i == 0:  # 直接加入第一个激光束，做为一个segment的起点
                _start.append(_segs[i][1])
            else:  # 间隔大于１以及距离差大于0.1m认为反光条不连续，分开。
                if _segs[i][0] > 1 or _segs[i][2] > 0.1:
                    _start.append(_segs[i][1])
                    _end.append(_segs[i - 1][1])
        _end.append(_segs[_len - 1][1])

        # 过滤掉连续20条一下激光束的segments, 每个segment代表一各反光条
        self.segments = filter(lambda s: s[1]-s[0] > 20, zip(_start, _end))

    def calculate(self, x=0.0, y=0.0, theta=0.0):
        if len(self.segments) == 0:
            print 'empty ...'
            return

        for i in range(len(self.segments)):
            _seg = self.segments[i]  # 一条激光束
            # 最小二乘法拟合直线 Ax = y
            _A2 = np.array(map(lambda d, t: [math.sin(t), 1.0/d],
                               self.msg.ranges[_seg[0]:_seg[1]],
                               self.cells[_seg[0]:_seg[1]]))
            _y2 = np.array(map(lambda t: math.cos(t),
                               self.cells[_seg[0]:_seg[1]]))
            _p2 = np.dot(np.dot(
                np.linalg.inv(np.dot(_A2.T, _A2)), _A2.T), _y2.T)

            _ds = map(lambda t: _p2[1]/(math.cos(t)-_p2[0]*math.sin(t)),
                      self.cells[_seg[0]:_seg[1]])
            _T = np.array([[math.cos(theta), -math.sin(theta), x],
                           [math.sin(theta), math.cos(theta),  y],
                           [0, 0, 1]])
            _coords = np.array(
                map(lambda d, t: np.dot(
                    _T, np.array([d*math.cos(t), d*math.sin(t), 1]).T),
                    _ds, self.cells[_seg[0]:_seg[1]]))

            self.coords.append(_coords)
            _x1 = _coords[0][0]
            _y1 = _coords[0][1]
            _x2 = _coords[-1][0]
            _y2 = _coords[-1][1]
            self.lens.append(math.sqrt((_x2-_x1)**2 + (_y2-_y1)**2))
            self.centrals.append([(_x2+_x1)/2.0, (_y2+_y1)/2.0])
            self.quarters.append([_x1/4.0+_x2*3.0/4.0, _y1/4.0+_y2*3.0/4.0])
            self.three_quarters.append(
                [_x1*3.0/4.0+_x2/4.0, _y1*3.0/4.0+_y2/4.0])


count = 0
ranges = []
intensities = []
gmsg = dict()

qs = []
tqs = []
times = 200

angle = 0.99742564272  # * math.pi / 180.0


def scan(msg):
    global qs, tqs, count, times, gmsg, angle
    gmsg = msg
    gmsg.ranges = list(msg.ranges)
    gmsg.intensities = list(msg.intensities)
    if count < times:
        _ref = reflector(gmsg)
        _ref.recognise()

        if listener.canTransform('map', gmsg.header.frame_id, rospy.Time(0)):
            _t, _r = listener.lookupTransform(
                'map', gmsg.header.frame_id, rospy.Time(0))
            _ref.calculate(x=_t[0], y=_t[1], theta=_r[3])
            qs.append(_ref.quarters)
            tqs.append(_ref.three_quarters)
    count = count + 1


def scan_process(msg):
    global count, ranges, intensities, gmsg
    if count == 0:
        intensities = list(msg.intensities)
        ranges = list(msg.ranges)
        gmsg = msg
        gmsg.ranges = list(msg.ranges)
        gmsg.intensities = list(msg.intensities)
    count = count + 1


rospy.init_node('landmark_localization')
#scan = rospy.Subscriber('scan', LaserScan, scan_process, queue_size=10)
scan = rospy.Subscriber('scan', LaserScan, scan, queue_size=10)
listener = TransformListener()
broadcaster = TransformBroadcaster()
stop_amcl_tf = rospy.ServiceProxy('stop_amcl_tf', Empty)

rate = rospy.Rate(100)
# rospy.spin()


def test():
    if count > 0:
        _ref = reflector(gmsg)
        _ref.recognise()
        _ref.calculate()
        _xys = np.array(map(lambda r, t: [r*math.cos(t), r*math.sin(t)],
                            _ref.msg.ranges, _ref.cells)).T

        """
        # ret = map(lambda d,i,t: math.sqrt(d*(abs(t)+2*math.pi))*i,
        #           _ref.msg.ranges, _ref.msg.intensities, _ref.cells)
        def _func(d):
            if d >= 10.0:
                return 0
            else:
                return math.sqrt(d / 10.0)
        ret = map(lambda d,i: _func(d)*i,
                  _ref.msg.ranges, _ref.msg.intensities)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(range(len(ret)), ret, _ref.msg.intensities, c='green', s=5)
#        plt.plot(range(len(ret)), ret, 'b*')
        for i in range(len(_ref.segments)):
            _seg = _ref.segments[i]
            plt.plot(range(_seg[0], _seg[1]), ret[_seg[0]:_seg[1]], 'r-')
            print _ref.lens[i]
        """
        # fig = plt.figure()
        # ax = Axes3D(fig)
        # ax.scatter(-_xys[1], _xys[0], _ref.msg.intensities, c='green', s=5)
        for i in range(len(_xys[0])):
            if (_ref.msg.intensities[i] > 900.0):
                plt.plot(-_xys[1][i], _xys[0][i], 'b+')
            else:
                plt.plot(-_xys[1][i], _xys[0][i], 'g+')

        for i in range(len(_ref.coords)):
            print _ref.lens[i], '(', _ref.centrals[i][0], _ref.centrals[i][1], ')'
            # print _ref.poses[i]
            _cs = np.array(_ref.coords[i]).T
            plt.plot(-_cs[1],  _cs[0],  'r-')
            plt.plot(-_ref.centrals[i][1], _ref.centrals[i][0], 'k*', linewidth=5)
            plt.plot(-_ref.quarters[i][1], _ref.quarters[i][0], 'y*', linewidth=5)
            plt.plot(-_ref.three_quarters[i][1],
                     _ref.three_quarters[i][0], 'g*', linewidth=5)
            # plt.plot(_ref.poses[i][1], _ref.poses[i][0], 'k^', linewidth=5)

        plt.show()

        """
        for i in xrange(len(ranges)):
            if ranges[i] == 'inf' or ranges[i] > 40.0:
                ranges[i] = 0
            if intensities[i] == 'inf':
                intensities[i] = 0
        _r = 0.0
        _d = 0.0
        for i in xrange(len(ranges)):
            _r = _r + ranges[i]
            _d = _d + intensities[i]
        for i in xrange(len(ranges)):
            ranges[i] = ranges[i] / _r
            intensities[i] = intensities[i] / _d

        # fig = plt.figure()
        # ax = Axes3D(fig)
        # ax.scatter(range(len(ranges)), ranges, intensities, c='green', s=5)
        plt.plot(range(len(ranges)), intensities, 'r+')
        plt.plot(range(430, 467, 1), intensities[430:467], 'b*')
        plt.show()
        """

measures = [
    Mark(np.array([49.75642986,11.10541091]),np.array([49.62505937,11.12308505])),
    Mark(np.array([50.85421708,8.76559817]),np.array([50.87316277, 8.89563538])),
    Mark(np.array([50.76547489,7.99009675]),np.array([50.78058415, 8.12313701])),
    Mark(np.array([50.4461705 ,6.66112879]),np.array([50.45722551, 6.79148545])),
    Mark(np.array([47.60213477,1.21493845]),np.array([47.90844402, 0.98137164]))
]

# 已计算好的反光条位置, 由四分之一和四分之三点来表示的直线形式
measures = [
    Mark(np.array([50.59889368,10.92137599]),np.array([50.46889726,10.94739645])),
    Mark(np.array([51.5446777 , 8.51582013]),np.array([51.57207954, 8.64441907])),
    Mark(np.array([51.40656683, 7.74751711]),np.array([51.43027119, 7.87931407])),
    Mark(np.array([51.00286104, 6.44152174]),np.array([51.02226561, 6.57089433])),
    Mark(np.array([47.81589156, 1.18862327]),np.array([48.10661774, 0.93555304]))
]

old = 0

# _pose = np.array([49.3437538181, 8.21690806005])
_pose = np.array([50.0, 8.3])
angle = angle + 0.03
while not rospy.is_shutdown():
    # test()

    if count % 2 == 1 and count > old:
        old = count
        _ref = reflector(gmsg)
        _ref.recognise()
        _ref.calculate(x=49.3437538181, y=8.21690806005, theta=angle)

        if listener.canTransform('map', gmsg.header.frame_id, rospy.Time(0)):
            _t, _r = listener.lookupTransform(
                'map', gmsg.header.frame_id, rospy.Time(0))
            _pose = np.array([_t[0], _t[1]])
            angle = _r[3]  # * math.pi / 180.0

        _len = len(_ref.quarters)
        if _len == 0:
            continue

        _calc_pose = []
        for j in xrange(1, _len):
            _m = Mark(
                np.array(_ref.quarters[j]), np.array(_ref.three_quarters[j]))
            for i in xrange(len(measures)):
                _f = measures[i].equals2(_m)
                if _f:
                    _ret = _m.mean(measures[i])
                    if _ret.rt[0] > math.pi:
                        break

                    _T = np.array(
                        [[math.cos(_ret.rt[0]), -math.sin(_ret.rt[0])],
                         [math.sin(_ret.rt[0]),  math.cos(_ret.rt[0])]])
                    # __p = np.dot(_T, (_pose - _m.central + _ret.rt[1:]).T)
                    # _calc_pose.append(__p + _m.central)
                    __p = np.dot(_T, (_pose - _m.central).T)
                    _calc_pose.append(__p + measures[i].central)
                    break
        _len = len(_calc_pose)
        if 0 < _len:
            for i in xrange(1, _len):
                _calc_pose[0] = _calc_pose[0] + _calc_pose[i]
            _calc_pose[0] = _calc_pose[0] / _len
            print _calc_pose[0], _calc_pose[0] - _pose

    else:
        rate.sleep()
    continue

    if count >= 400:
        marks = []
        for i in xrange(len(qs)):
            for j in xrange(len(qs[i])):
                _m = Mark(np.array(qs[i][j]), np.array(tqs[i][j]))
                _f = False
                for k in xrange(len(marks)):
                    _f = marks[k].update(_m)
                    if _f:
                        break
                    else:
                        pass
                if not _f:
                    marks.append(_m)
                else:
                    pass

        for i in xrange(len(marks)):
            if marks[i].n > 100:
                # print marks[i].mean().central, marks[i].n
                print marks[i].mean(Mark()).quarter
                print marks[i].mean(Mark()).three_quarter

        break
    else:
        rate.sleep()
