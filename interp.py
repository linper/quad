import numpy as np
import math
from consts import *


def drop_time(h):
    return abs(h - MAX_DIP) / ((0.25 * T_RAD) / MIN_PERIOD) + MIN_PERIOD


def fill_diffs(lst):
    time = np.array([float(i.t) for i in lst])
    data_x = np.array([float(i.x) for i in lst])
    data_y = np.array([float(i.y) for i in lst])
    data_z = np.array([float(i.z) for i in lst])

    dx = akima(time, data_x)
    dy = akima(time, data_y)
    dz = akima(time, data_z)

    for i in range(len(lst)):
        if lst[i].dx is None:
            lst[i].dx = dx[i]

        if lst[i].dy is None:
            lst[i].dy = dy[i]

        if lst[i].dz is None:
            lst[i].dz = dz[i]


def line_cof(x1, y1, x2, y2):
    fi = 0.0
    if x2 == x1:
        fi = 0.0000001

    k = (y2 - y1) / (x2 - x1 + fi)
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
            if X[index] - X[j] == 0:
                a = 0
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


# def get_legs_normal(arr):
#     has_normal = False
#     for i in range(4):
#         if not grounded_legs[cw_seq[i]] or not grounded_legs.__contains__(False):
#             a = np.array(arr[cw_seq[i - 1]]) - np.array(
#                 arr[cw_seq[i - 2]])
#             b = np.array(arr[cw_seq[i - 1]]) - np.array(
#                 arr[cw_seq[i - 3]])
#             c = get_cross_product(b, a) * np.array([-1, -1, 1])
#             return np.array([1, -1, 1]) * strech_vector_to(c, 1)
#     if not has_normal:
#         return np.array([0, 0, -1])


def get_vectors_cosine(a, b):
    la = np.linalg.norm(a)
    lb = np.linalg.norm(b)
    if la == 0 or lb == 0:
        return 0
    else:
        return np.dot(a, b) / (la * lb)


def get_vectors_angle(a, b):
    return math.acos(get_vectors_cosine(a, b))


def get_cross_product(a, b):
    # return np.cross(a, b)
    return np.array([a[1] * b[2] - a[2] * b[1],  a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def get_rotation_matrix_from_two_vectors(a, b):
    a = strech_vector_to(a, 1)
    b = strech_vector_to(b, 1)
    phi = math.acos(get_vectors_cosine(a, b))
    axis = strech_vector_to(get_cross_product(a, b), 1)
    matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    rcos = math.cos(phi)
    rsin = math.sin(phi)
    matrix[0][0] = rcos + axis[0] * axis[0] * (1 - rcos)
    matrix[1][0] = axis[2] * rsin + axis[1] * axis[0] * (1 - rcos)
    matrix[2][0] = -axis[1] * rsin + axis[2] * axis[0] * (1 - rcos)
    matrix[0][1] = -axis[2] * rsin + axis[0] * axis[1] * (1 - rcos)
    matrix[1][1] = rcos + axis[1] * axis[1] * (1 - rcos)
    matrix[2][1] = axis[0] * rsin + axis[2] * axis[1] * (1 - rcos)
    matrix[0][2] = axis[1] * rsin + axis[0] * axis[2] * (1 - rcos)
    matrix[1][2] = -axis[0] * rsin + axis[1] * axis[2] * (1 - rcos)
    matrix[2][2] = rcos + axis[2] * axis[2] * (1 - rcos)

    return matrix


def strech_vector_to(v, lenght):
    lv = np.linalg.norm(v)
    if lv != 0:
        return v * (lenght / lv)
    else:
        return np.array([0, 0, 0])


def vector_projection(from_vec, to_vec):
    return (np.dot(from_vec, to_vec)) / np.linalg.norm(to_vec)
