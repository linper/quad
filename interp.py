import numpy as np
import math
from common import *
from consts import *


def drop_time(h):
    return (h - MAX_DIP) / ((0.25 * T_RAD) / MIN_PERIOD)


def fill_diffs(lst):
    time = np.array([float(i.t) for i in lst])
    data_x = np.array([float(i.x) for i in lst])
    data_y = np.array([float(i.y) for i in lst])

    dx = akima(time, data_x)
    dy = akima(time, data_y)

    for i in range(len(lst)):
        if lst[i].dx is None:
            lst[i].dx = dx[i]

        if lst[i].dy is None:
            lst[i].dy = dy[i]


def line_cof(x1, y1, x2, y2):
    k = (y2 - y1) / (x2 - x1)
    b = y1 - k * x1
    return k, b


def intersect(k1, b1, k2, b2):
    if round(k1, 4) == round(k2, 4):
        return False, 0, 0

    x = (b2 - b1) / (k1 - k2)
    y = k1 * x + b1
    return True, x, y


def sect_intersect(x11, y11, x12, y12, x21, y21, x22, y22):
    k1, b1 = line_cof(x11, y11, x12, y12)
    k2, b2 = line_cof(x21, y21, x22, y22)
    incl, x, y = intersect(k1, b1, k2, b2)
    if not incl:
        return incl, x, y

    incl = min(x11, x12) < x < max(x11, x12) and \
           min(x21, x22) < x < max(x21, x22) and \
           min(y11, y12) < y < max(y11, y12) and \
           min(y21, y22) < y < max(y21, y22)
    return incl, x, y


def dist(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def akima(X, Y) -> np.ndarray:
    Xi = X[1:len(X) - 1]
    Yi = Y[1:len(Y) - 1]
    XM1 = X[:len(X) - 2]
    XP1 = X[2:]
    YM1 = Y[:len(Y) - 2]
    YP1 = Y[2:]

    AM1 = (Xi - XP1) / ((XM1 - Xi) * (XM1 - XP1)) * YM1
    Ai = (2 * Xi - XP1 - XM1) / ((Xi - XM1) * (Xi - XP1)) * Yi
    AP1 = (Xi - XM1) / ((XP1 - XM1) * (XP1 - Xi)) * YP1
    A = np.empty(shape=X.shape, dtype=X.dtype)
    A[1:len(A) - 1] = AM1 + Ai + AP1
    A[0] = A[1]
    A[-1] = A[-2]
    return A


def f(x):
    return math.log(x, math.e) / (math.sin(2 * x) + 1.5) - x / 7


def lagrange(X, Y, visible_x):
    L = np.zeros(shape=visible_x.shape, dtype=np.float32)
    _L = np.empty(shape=visible_x.shape, dtype=np.float32)
    for i in range(len(X)):
        _L.fill(1.0)
        for j in range(len(X)):
            if j != i:
                _L = _L * ((visible_x - X[j]) / (X[i] - X[j]))
        L = L + _L * Y[i]
    return L


def lagrange_single(X, index, visible_x):
    L = np.ones(shape=visible_x.shape, dtype=np.float32)
    for j in range(len(X)):
        if j != index:
            L = L * ((visible_x - X[j]) / (X[index] - X[j]))
    return L


def d_lagrange_single(X, index):
    dL = 0.0
    for j in range(len(X)):
        if j != index:
            dL = dL + (1 / (X[index] - X[j]))
    return dL


def hermite(X, Y, dY, n):
    visible_x = np.linspace(X[0], X[1], n)

    # x = visible_x-X[0]
    # d = X[1]-X[0]
    # U1 = 1-3*(x/d)**2 + 2*(x/d)**3
    # U2 = 3*(x/d)**2 - 2*(x/d)**3
    # V1 = x-2*(x**2)/d + (x**3)/d**2
    # V2 = -(x**2)/d + (x**3)/d**2

    L1 = lagrange_single(X, 0, visible_x)
    dL1 = d_lagrange_single(X, 0)
    U1 = (1 - 2 * dL1 * (visible_x - X[0])) * L1 ** 2
    V1 = (visible_x - X[0]) * L1 ** 2
    L2 = lagrange_single(X, 1, visible_x)
    dL2 = d_lagrange_single(X, 1)
    U2 = (1 - 2 * dL2 * (visible_x - X[1])) * L2 ** 2
    V2 = (visible_x - X[1]) * L2 ** 2

    interp_values = U1 * Y[0] + V1 * dY[0] + U2 * Y[1] + V2 * dY[1]
    return interp_values


def connect_splines(X, Y, dY, n):
    visible_y = []
    for i in range(len(X) - 1):
        X_pair = X[i:i + 2]
        Y_pair = Y[i:i + 2]
        dY_pair = dY[i:i + 2]
        spline_interval = hermite(X_pair, Y_pair, dY_pair, n)
        if len(visible_y) == 0:
            visible_y.extend(spline_interval)
        else:
            visible_y.extend(spline_interval[1:])
    return visible_y


def connect_splines2(X, Y, dY, t):
    visible_y = []
    for i in range(len(X) - 1):
        X_pair = X[i:i + 2]
        Y_pair = Y[i:i + 2]
        dY_pair = dY[i:i + 2]
        n = int(math.ceil((t[i + 1] - t[i]) / STEP))
        # if (n < 0):
        #     a = 9
        spline_interval = hermite(X_pair, Y_pair, dY_pair, n + 1)
        if len(visible_y) == 0:
            visible_y.extend(spline_interval)
        else:
            visible_y.extend(spline_interval[1:])
    return visible_y


def get_spline(X, Y, dY, n):
    X_pair = X[:2]
    Y_pair = Y[:2]
    dY_pair = dY[:2]
    spline_interval = hermite(X_pair, Y_pair, dY_pair, n)
    return spline_interval


def interp_linear(x0, x1, pts: list):
    pts_arr = np.array(pts)
    # pts_arr = pts_arr - np.min(pts_arr)
    pts_arr = pts_arr / np.max(pts_arr)

    mul = x1 - x0
    ret = [(p * mul) + x0 for p in pts_arr]
    return ret


def connect_times(t):
    visible_y = []
    for i in range(len(t) - 1):
        n = int(math.ceil((t[i + 1] - t[i]) / STEP))
        time_steps = np.linspace(t[i], t[i + 1], n + 1, dtype=np.float32)
        if len(visible_y) == 0:
            visible_y.extend(time_steps)
        else:
            visible_y.extend(time_steps[1:])
    return visible_y
