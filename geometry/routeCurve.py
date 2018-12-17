#!/usr/bin/python
import math
import numpy as np
import matplotlib.pyplot as plt


class routeCurve:
    Mk = np.array([[1.0,    0.0,   0.0,   0.0,  0.0, 0.0],
                   [-5.0,   5.0,   0.0,   0.0,  0.0, 0.0],
                   [10.0, -20.0,  10.0,   0.0,  0.0, 0.0],
                   [-10.0, 30.0, -30.0,  10.0,  0.0, 0.0],
                   [5.0,  -20.0,  30.0, -20.0,  5.0, 0.0],
                   [-1.0,   5.0, -10.0,  10.0, -5.0, 1.0]])
    Hk = np.array([[1.0,   0.0,    0.0,    0.0,    0.0,   0.0],
                   [1.0, 1/5.0,    0.0,    0.0,    0.0,   0.0],
                   [1.0, 2/5.0, 1/20.0,    0.0,    0.0,   0.0],
                   [0.0,   0.0,    0.0, 1/20.0, -2/5.0,   1.0],
                   [0.0,   0.0,    0.0,    0.0, -1/5.0,   1.0],
                   [0.0,   0.0,    0.0,    0.0,    0.0,   1.0]])

    def __init__(self, xs, ys):
        if len(xs) < 3 or len(xs) != len(ys):
            print 'Invalid data!'
            return

        self.US = np.array([i / 50.0 for i in range(51)])
        self.coords = np.array([xs, ys])
        self.PS = self.controlPoints()
        self.Curve = map(
            lambda cs: self.bezier5(np.array([cs[0], cs[1]])), self.PS)
        cs = reduce(lambda x, y: x+y, self.Curve)
        self.XS = map(lambda x: x[0], cs)
        self.YS = map(lambda x: x[1], cs)

    def curvePoints(self):
        return self.XS, self.YS

    def controlPoints(self):
        diff = np.diff(self.coords)

        def corr(x1, y1, x2, y2):
            tpi = 2*math.pi
            theta = math.atan2(y2, x2) - math.atan2(y1, x1)
            if theta < -math.pi:
                theta = theta + tpi
            if theta > math.pi:
                theta = theta - tpi
            return theta / 2.0

        def difference(data):
            init = data[:, 1:]
            tail = data[:, :-1]
            coef = [0] + map(lambda x1, y1, x2, y2: corr(x1, y1, x2, y2),
                             init[0], init[1], tail[0], tail[1])
            return coef

        def trans(xy, t):
            R = np.array([[math.cos(t), -math.sin(t)],
                          [math.sin(t), math.cos(t)]])
            return np.dot(R, xy.T)

        coef = difference(diff)
        X = map(lambda xy, t: trans(xy, t)[0], diff.T, coef)
        Y = map(lambda xy, t: trans(xy, t)[1], diff.T, coef)
        X.append(X[-1])
        Y.append(Y[-1])

        tdiff = np.diff(np.array([X, Y]))
        tcoef = difference(tdiff)
        coef.append(0)
        tcoef.append(0)

        def process(p0, t0, a0, a1, t1, p1):
            _deltP = p1 - p0
            _t0 = trans(_deltP,  t0)
            _t1 = trans(_deltP, -t1)

            _deltT = _t1 - _t0
            _a0 = trans(_deltT,  a0)
            _a1 = trans(_deltT, -a1)

            _pp = np.dot(self.Hk,
                         np.array([p0, _t0/4.0, _a0/4.0, _a1/4.0, _t1/4.0, p1]))
            return map(lambda p: p[0], _pp), map(lambda p: p[1], _pp)

        return map(lambda p0, t0, a0, a1, t1, p1:
                   process(p0, t0, a0, a1, t1, p1),
                   self.coords[:, :-1].T,
                   coef[:-1],  tcoef[:-1], tcoef[1:], coef[1:],
                   self.coords[:, 1:].T)

    def uv(self, u):
        return np.array([u ** i for i in range(6)])

    def du(self, u):
        return np.array([0, 1.0, u/2.0, (u**2)/3.0, (u**3)/4.0, (u**4)/5.0])

    def d2u(self, u):
        return np.array([0, 0, 0.5, u/6.0, (u**2)/12.0, (u**3)/20.0])

    def bezier5(self, cs):
        return map(lambda u: np.dot(
            np.dot(self.uv(u), self.Mk), cs.T), self.US)

    def bezier5_du(self, cs):
        return map(lambda u: np.dot(
            np.dot(self.du(u), self.Mk), cs.T), self.US)

    def bezier5_d2u(self, cs):
        return map(lambda u: np.dot(
            np.dot(self.d2u(u), self.Mk), cs.T), self.US)

    def curvature(self):
        _cs = np.array([self.XS, self.YS])
        _ti = np.diff(_cs, 1)
        _ai = np.diff(_cs, 2)

        _tx = [0] + _ti[0].tolist()
        _ty = [0] + _ti[1].tolist()
        _ax = [0] + _ai[0].tolist() + [0]
        _ay = [0] + _ai[1].tolist() + [0]

        ks = []
        for i in range(len(self.XS)):
            # a = abs(_tx[i] * _ay[i] - _ax[i] * _ty[i])
            a = (_tx[i] * _ay[i] - _ax[i] * _ty[i])
            b = (_tx[i] ** 2 + _ty[i] ** 2) ** 1.5
            if b == 0:
                ks.append(0)
            else:
                ks.append(a / b)
        return ks


def test():
    # theta = math.pi / 4.0
    def rotation(theta, xs, ys):
        c = math.cos(theta)
        s = math.sin(theta)
        xs = map(lambda x, y: x*c-y*s, xs, ys)
        ys = map(lambda x, y: x*s+y*c, xs, ys)
        return xs, ys

    # xs = [math.sin(i / (math.pi)) for i in range(100)]
    # ys = [1.5*math.sin(i / (2 * math.pi)) for i in range(100)]
    xs = [26.448, 26.448, 26.448, 26.448, 26.448, 26.448, 26.498,
          26.398, 25.448, 24.448, 23.448, 22.448, 21.448, 20.448,
          19.448, 18.448, 17.448, 16.448, 15.448, 14.448, 13.448,
          12.448, 11.448, 10.448, 9.448, 8.448, 7.448, 6.448, 5.448,
          4.448, 3.448, 2.498, 2.398, 2.448, 2.448, 2.448, 2.448,
          2.448, 2.448, 2.448, 2.448, 2.448, 2.448, 2.448, 2.398,
          2.498, 3.448, 4.448, 5.448, 6.448, 7.448, 8.448, 9.448,
          10.448, 11.448, 12.448, 13.448, 14.448, 15.448, 16.448,
          17.448, 18.448, 19.448, 20.448, 21.448, 22.448, 23.448,
          24.448, 25.448, 26.398, 26.498, 26.448, 26.448, 26.448, 26.448, 26.448]

    ys = [8, 9, 10, 11, 12, 13, 14, 14.95, 15.05, 15, 15, 15, 15,
          15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
          15, 15, 15, 15.05, 14.95, 14, 13, 12, 11, 10, 9, 8, 7, 6,
          5, 4, 3, 2, 1.55, 1.45, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5,
          1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5,
          1.5, 1.5, 1.45, 1.55, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5]
    # xs, ys = rotation(math.pi / 4.0, xs, ys)

    # rc = routecurve(xs[:], ys[:])

    # plt.subplot(121)
    # for i in range(len(rc.ps)):
    #     plt.plot(rc.ps[i][0], rc.ps[i][1], 'y-..', linewidth=1)
    # plt.plot(xs, ys, 'k--*', linewidth=1)
    # plt.plot(rc.xs, rc.ys, 'b-', label='bezier curve', linewidth=2)

    # plt.subplot(122)
    # ks = rc.curvature()
    # plt.plot(rc.xs, ks, 'r-', label='bezier curve', linewidth=1)
    # plt.show()


xs = [i + 1 for i in range(10)]
ys = [math.cos(i * 70 / math.pi) for i in xs]
xs = [math.sin(i / (math.pi)) for i in range(30)]
ys = [1.5*math.sin(i / (2 * math.pi)) for i in range(30)]

xs = [4*math.sin(2*i / (math.pi)) for i in range(25)]
ys = [3*math.sin(2*i / (2 * math.pi)) for i in range(25)]
rc = routeCurve(xs, ys)

plt.plot(xs, ys, '-')
plt.plot(rc.XS, rc.YS, 'b-', label='Bezier Curve', linewidth=3)
plt.plot(rc.XS, rc.YS, 'r-', label='Bezier Curve', linewidth=2)
#plt.plot(rc.curvature(), 'b-')
plt.show()


"""
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from path_msgs.msg import FollowPathActionGoal, FollowPathActionResult, DirectionalPath
from std_srvs.srv import EmptyRequest, Empty
from tf import TransformListener

xs = []
ys = []


def path_cb(msg):
    # msg = Path()
    print 'received path'
    if len(msg.poses) > 1:
        try:
            trans, rot = tf_.lookupTransform('map', 'base_link', rospy.Time(0))
            pose_1_to_robot = [trans[0] - msg.poses[0].pose.position.x,
                               trans[1] - msg.poses[0].pose.position.y]
            pose_1_to_pose_2 = [msg.poses[1].pose.position.x - msg.poses[0].pose.position.x,
                                msg.poses[1].pose.position.y - msg.poses[0].pose.position.y]
            if pose_1_to_robot[0] ** 2 + pose_1_to_robot[1] ** 2 > 0.2 and\
                pose_1_to_robot[0] * pose_1_to_pose_2[0] + pose_1_to_robot[1] * pose_1_to_pose_2[1] < 0:
                print 'add robot pose'
                xs.append(trans[0])
                ys.append(trans[1])
        except e:
            print e
    for pose in msg.poses:
        xs.append(pose.pose.position.x)
        ys.append(pose.pose.position.y)
    start_cb(None)


def waypoint_cb(msg):
    # msg = PoseStamped()
    print 'received a waypoint'
    xs.append(msg.pose.position.x)
    ys.append(msg.pose.position.y)


def start_cb(msg):
    print 'start'
    global xs, ys
    print 'xs: ', xs, ys
    if len(xs) == 2:
        if xs[0] == xs[1]:
            xs.append(xs[0])
            _y_new = (ys[1] + ys[0]) / 2
            ys.insert(1, _y_new)
        else:
            _k_line = ((ys[1] - ys[0]) / (xs[1] - xs[0]))
            _b_line = ys[0] - _k_line * xs[0]
            _x_new = (xs[1] - xs[0]) / 2+xs[0]
            _y_new = _k_line * _x_new + _b_line
            xs.insert(1, _x_new)
            ys.insert(1, _y_new)
            print "_k_line :", _k_line
            print "_b_line :", _b_line
            print "_x_new :", _x_new
            print "_y_new :", _y_new
        print "len<2 xs :", xs
        print "len<2 ys :", ys

    if len(xs) > 2:
        rc = routeCurve(xs, ys)
        path_goal = FollowPathActionGoal()
        path_goal.header.stamp = rospy.Time.now()
        path_goal.goal_id.stamp = rospy.Time.now()
        path_goal.goal.follower_options.init_mode = 0;
p        path_goal.goal.follower_options.velocity = 0.8;
        path_goal.goal.path.header.stamp = rospy.Time.now()
        path_goal.goal.path.header.frame_id = "map";
        path_goal.goal.path.paths.append(DirectionalPath())

        for i in xrange(len(rc.XS)):
            pose = PoseStamped()
            pose.header.frame_id = "/map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = rc.XS[i]
            pose.pose.position.y = rc.YS[i]
            pose.pose.orientation.w = 1.0
            path_goal.goal.path.paths[0].poses.append(pose)

        path_goal.goal.path.paths[0].forward = True
        goal_pub.publish(path_goal)

    else:
        print "not enough waypoints."
    xs = []
    ys = []


def result_cb(result):
    print 'result: ',result.status.text
    if result.status.status == result.status.ABORTED or \
                    result.status.status == result.status.REJECTED:
        rospy.logwarn(result.status.text)
    elif result.status.status == result.status.SUCCEEDED:
        rospy.loginfo('Goal reached')
        rospy.wait_for_service('goal_reach')
        try:
            from time import sleep
            sleep(0.25)
            srv = EmptyRequest()
            resp = goal_reach_srv(srv)
            rospy.loginfo('Call service.')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__':
    rospy.init_node('global_path')
    waypoint_sub = rospy.Subscriber('move_base_simple/waypoint', PoseStamped, waypoint_cb, queue_size=10)
    path_sub = rospy.Subscriber('navigation/ideal_path', Path, path_cb, queue_size=10)
    start_sub = rospy.Subscriber('clicked_point', PointStamped, start_cb, queue_size=10)
    result_sub = rospy.Subscriber('follow_path/result', FollowPathActionResult, result_cb, queue_size=10)

    goal_pub = rospy.Publisher('follow_path/goal', FollowPathActionGoal, queue_size=10)
    goal_reach_srv = rospy.ServiceProxy('goal_reach', Empty)

    tf_ = TransformListener()
    rospy.spin()
"""