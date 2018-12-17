# coding=utf-8
import math
import numpy as np

import rospy
from tf import TransformListener, TransformBroadcaster, transformations
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from path_msgs.msg import FollowPathActionGoal, FollowPathActionResult, DirectionalPath

cell = 0.25 * math.pi / 180.0
angle_table = map(lambda i: float((448 - i)) * cell, xrange(897))
ctable = map(lambda t: math.cos(t), angle_table)
stable = map(lambda t: math.sin(t), angle_table)

rospy.init_node('features')
listener = TransformListener()
broadcaster = TransformBroadcaster()
pub = rospy.Publisher('feats', FollowPathActionGoal, queue_size=10)


def dot(v):
    return np.dot(v, v.T)


def norm(v):  # 计算向量范数
    return np.linalg.norm(v)


def feature(msg):
    path_goal = FollowPathActionGoal()
    path_goal.header.stamp = rospy.Time.now()
    path_goal.goal_id.stamp = rospy.Time.now()
    path_goal.goal.follower_options.init_mode = 0;
    path_goal.goal.follower_options.velocity = 0.8;
    path_goal.goal.path.header.stamp = rospy.Time.now()
    path_goal.goal.path.header.frame_id = "map";
    path_goal.goal.path.paths.append(DirectionalPath())

    _ranges = np.array(msg.ranges)
    _intensity = np.array(msg.intensities)
    for i in range(len(_ranges)):  # 设置有效检测距离的范围
        if _ranges[i] < 1.0 or _ranges[i] > 5.0:
            _ranges[i] = 0
        if _intensity[i] < 600:
            _ranges[i] = 0

    _ddxy, _ration = [], []
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

    _diff = [0] + map(
        lambda v, w, n: abs(np.cross((v+v), (v+w))*(n)),
        _ddxy[:-1], _ddxy[1:], _ration[1:])

    _segs = []
    __thr, __int = 2, 2
    __s, __e = 0, 0
    for i in range(len(_diff)):
        if _ranges[i+1] == 0.0:
            _diff[i] = float("inf")
        if _diff[i] > __thr:
            __e = max(0, i-1)
            if __e - __s > __int and __e - __s < 10:
                _segs.append([__s+2, __e+2])
            __s = i

    for seg in _segs:
        __x, __y = 0, 0
        s, e = seg
        c = e - s + 1
        while s <= e:
            __x = __x + _ranges[s] * stable[s]
            __y = __y + _ranges[s] * ctable[s]
            s = s + 1
        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = __x / c
        pose.pose.position.y = __y / c
        pose.pose.orientation.w = 1.0
        path_goal.goal.path.paths[0].poses.append(pose)

    if len(_segs) > 0:
        pub.publish(path_goal)


def callback(msg):
    feature(msg)


scan = rospy.Subscriber('scan', LaserScan, callback, queue_size=10)
r = rospy.Rate(30)
while not rospy.is_shutdown():
    r.sleep()
