import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def differentialDrive(vr, vl, b):
    u = (vr + vl) / 2.0
    if (vr == 0 and vl == 0) or (b == 0):
        return [u, 0, 0]
    w = (vr - vl) / (2.0 * b)
    k = w / u
    return [u, w, k]



def test1():
    vrs = [float(i) / 50.0 for i in range(51)]
    vls = vrs

    fig = plt.figure()
    ax = Axes3D(fig)

    for i in range(len(vrs)):
        us = []
        ws = []
        ks = []
        for j in range(len(vls)):
            u, w, k = differentialDrive(vrs[i], vls[j], 0.3)
            us.append(u)
            ws.append(w)
            ks.append(k)
        rs = [vrs[i] for k in range(len(vrs))]
        ax.scatter(vls, rs, ks, c='b', s=15)
        ax.scatter(vls, rs, us, c='g', s=5)
        ax.scatter(vls, rs, ws, c='r', s=5)
    # ax.scatter(vls, vrs, [0 for i in range(len(vrs))], c='k', s=10)

    # plt.plot(us, ws, 'r')
    # plt.plot(ws, 'b')
    # plt.plot(ks, 'g')

    # ax.scatter(us, ws, ks, '--+', s=5)
    # ax.scatter(vrs, vls, ks, '--+', s=5)

    plt.show()


def curve1(t, a, b):
    zs = np.linspace(-5 * np.pi, 5 * np.pi, 1000)
    xs = a * np.cos(zs)
    ys = a * np.sin(zs)
    zs = b * zs
    return xs, ys, zs


def length(xs, ys, zs):
    dxs = np.diff(xs)
    dys = np.diff(ys)
    dzs = np.diff(zs)
    ls = map(lambda x, y, z: math.sqrt(x**2 + y**2 + z**2), dxs, dys, dzs)
    ds = []
    ds.append(0)
    for i in range(len(ls)):
        ds.append(ls[i] + ds[-1])
    return ds


def measurement(xs, ys, zs):
    dxs = np.diff(xs, 1)
    dys = np.diff(ys, 1)
    dzs = np.diff(zs, 1)

    ds = np.array(
        map(lambda x, y, z: math.sqrt(x**2 + y**2 + z**2), dxs, dys, dzs))
    dxs = (dxs / ds)[2:]
    dys = (dys / ds)[2:]
    dzs = (dzs / ds)[2:]

    ddxs = np.diff(xs, 2)
    ddys = np.diff(ys, 2)
    ddzs = np.diff(zs, 2)

    d2s = ds[1:] ** 2
    ddxs = (ddxs / d2s)[1:]
    ddys = (ddys / d2s)[1:]
    ddzs = (ddzs / d2s)[1:]

    d3s = ds[2:] ** 3
    d3xs = np.diff(xs, 3) / d3s
    d3ys = np.diff(ys, 3) / d3s
    d3zs = np.diff(zs, 3) / d3s

    a = map(lambda x, y, z: x**2 + y**2 + z**2, dxs, dys, dzs)
    b = map(lambda x, y, z: x**2 + y**2 + z**2, ddxs, ddys, ddzs)
    c = map(lambda x, y, z, xx, yy, zz:
            x*xx+y*yy+z*zz, dxs, dys, dzs, ddxs, ddys, ddzs)

    curvature = map(lambda x, y, z: math.sqrt((x*y-z)/(x**3)), a, b, c)
    torsion = []
    for i in range(len(curvature)):
        M = np.array([[dxs[i],  dys[i],  dzs[i]],
                      [ddxs[i], ddys[i], ddzs[i]],
                      [d3xs[i], d3ys[i], d3zs[i]]])
        N = 1.0 / curvature[i] ** 2
        O = a[i] ** 3
        torsion.append(np.linalg.det(M)*N/O)

    return curvature, torsion


ts = [i for i in range(10)]
xs, ys, zs = curve1(ts, 3, 4)

fig = plt.figure()
ax = Axes3D(fig)
ax.plot(xs, ys, zs, '-')
# ax.plot(xs, ys, length(xs, ys, zs), '-')
c, t = measurement(xs, ys, zs)
ax.plot(c, ys[3:], zs[3:], '-', c='k')
ax.plot(xs[3:], t, zs[3:], '-', c='r')

plt.grid(True)
plt.show()
