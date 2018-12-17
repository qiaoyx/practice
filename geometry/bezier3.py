# coding=utf-8
import cv2
import math
import string
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Twist, Pose
import tf
import rospy


class Bezier3:
    coef = np.array([1, 3, 3, 1])

    def __init__(self, pose):
        self.start = pose
        self.end = PoseStamped()
        self.end.pose.orientation.w = 1.0

        rot = tf.transformations.quaternion_from_euler(0, 0, math.pi/6.0)
        self.p = PoseStamped()
        self.p.pose.position.x, self.p.pose.position.y = 0.3, 0.8
        self.p.pose.orientation.x, self.p.pose.orientation.y = rot[0], rot[1]
        self.p.pose.orientation.z, self.p.pose.orientation.w = rot[2], rot[3]

    def tvector(self, t):
        b = 1.0 - t
        t0 = b**3
        t1 = t * (b**2)
        t2 = (t**2) * b
        t3 = t ** 3
        return np.array([t0, t1, t2, t3])

    def ddtv(self, t):
        t0 = -6.0*t + 6.0
        t1 = 6.0*t - 4.0
        t2 = -6.0*t + 2.0
        t3 = 6.0*t
        return np.array([t0, t1, t2, t3])

    def dtv(self, t):
        t0 = -3.0*(t**2) + 6.0*t - 3.0
        t1 = 3.0*(t**2) - 4.0*t + 1.0
        t2 = -3.0*(t**2) + 2.0*t
        t3 = 3.0*(t**2)
        return np.array([t0, t1, t2, t3])

    def control_points(self):
        (x, y, theta) = self.xyt(self.start)
        c, s = math.cos(theta), math.sin(theta)
        trans = np.array([[c, -s], [s, c]])
        dt = math.sqrt(x*x + y*y)
        # p = trans.dot(np.array([abs(y)/2.0, 0.0])) + np.array([x, y])
        p = trans.dot(np.array([abs(dt)*0.25, 0.0])) + np.array([x, y])

        p1, p2 = PoseStamped(), PoseStamped()
        p1.pose.position.x = abs(dt)*0.5
        # p1.pose.position.y = -abs(y)/2.0
        p1.pose.orientation.w = 1.0
        p2.pose.position.x = p[0]
        p2.pose.position.y = p[1]
        p2.pose.orientation = self.start.pose.orientation
        return (p2, p1)

    def xyt(self, p):
        (R, P, Y) = tf.transformations.euler_from_quaternion(
            [p.pose.orientation.x, p.pose.orientation.y,
             p.pose.orientation.z, p.pose.orientation.w])
        return (p.pose.position.x, p.pose.position.y, Y)

    def curve(self, p0, p1, p2, p3):
        (x0, y0, t) = self.xyt(p0)
        (x1, y1, t) = self.xyt(p1)
        (x2, y2, t) = self.xyt(p2)
        (x3, y3, t) = self.xyt(p3)

        def trans(x, y, t):
            theta = math.pi/6.0
            c, s = math.cos(theta), math.sin(theta)
            return (x*c-y*s + self.p.pose.position.x,
                    x*s+y*c + self.p.pose.position.y,
                    theta+t)
        (x0, y0, t) = trans(x0, y0, t)
        (x1, y1, t) = trans(x1, y1, t)
        (x2, y2, t) = trans(x2, y2, t)
        (x3, y3, t) = trans(x3, y3, t)

        ts = np.array([i / 100.0 for i in xrange(101)])
        xs = map(lambda t: np.array([x0, x1, x2, x3]).dot(
            self.coef*self.tvector(t)), ts)
        ys = map(lambda t: np.array([y0, y1, y2, y3]).dot(
            self.coef*self.tvector(t)), ts)

        dxs = map(lambda t: np.array([x0, x1, x2, x3]).dot(
            self.coef*self.dtv(t)), ts)
        dys = map(lambda t: np.array([y0, y1, y2, y3]).dot(
            self.coef*self.dtv(t)), ts)

        ddxs = map(lambda t: np.array([x0, x1, x2, x3]).dot(
            self.coef*self.ddtv(t)), ts)
        ddys = map(lambda t: np.array([y0, y1, y2, y3]).dot(
            self.coef*self.ddtv(t)), ts)

        curvature = map(lambda dx, dy, ddx, ddy:
                        (dx*ddy - dy*ddx) / math.pow(dx*dx+dy*dy, 1.5),
                        dxs, dys, ddxs, ddys)

        dx = map(lambda x, y: x/(math.sqrt(x**2 + y**2)) * 0.02, dxs, dys)
        dy = map(lambda x, y: y/(math.sqrt(x**2 + y**2)) * 0.02, dxs, dys)

        fig = plt.figure()
        ax = fig.add_subplot()
        plt.plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'k*')
        plt.plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'k--')
        plt.plot(xs, ys, 'b-', linewidth=3)
        plt.plot(xs, curvature, 'y-', linewidth=1)

        plt.plot([self.p.pose.position.x, self.p.pose.position.x+1.],
                 [self.p.pose.position.y,
                  self.p.pose.position.y+math.tan(math.pi/6.0)], 'r--')
        plt.plot([self.p.pose.position.x],
                 [self.p.pose.position.y], 'k.')

        for i in xrange(len(xs)):
            plt.plot([xs[i], xs[i]+dx[i]], [ys[i], ys[i]+dy[i]], 'k.--',
                     linewidth=1)


# ps = [(1.5, 0.2, -0.1), (1.5, 0.2, 0.1),
#       (1.0, 0.1, -0.1), (2.0, 0.2, -0.2),
#       (0.5, 0.05, 0.05)]
ps = [(1.5, 0.1, -0.1)]
if __name__ == '__main__':
    for i in range(len(ps)):
        rot = tf.transformations.quaternion_from_euler(0, 0, math.pi+ps[i][2])
        _p = PoseStamped()
        _p.pose.position.x, _p.pose.position.y = ps[i][0], ps[i][1]
        _p.pose.orientation.x, _p.pose.orientation.y = rot[0], rot[1]
        _p.pose.orientation.z, _p.pose.orientation.w = rot[2], rot[3]
        print rot

        _b3 = Bezier3(_p)

        print _b3.xyt(_b3.start)
        (p1, p2) = _b3.control_points()
        _b3.end.pose.position.x = 0.35
        _b3.curve(_b3.start, p1, p2, _b3.end)

    plt.show()
