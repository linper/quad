import numpy as np
import math
from consts import *
from numba import njit


def drop_time(h):
    return abs(h - MAX_DIP) / ((0.25 * T_RAD) / MIN_PERIOD) + MIN_PERIOD


def get_d2_speed(d1, d2):
    S = np.linalg.norm(d2.pos - d1.pos)
    n_step = d2.ts - d1.ts
    vel = S / n_step
    return vel


def do_nothing():
    pass


def gradual_speed_func(d1, d2):
    vels_ps = np.linspace(d1.vel_ps, d2.vel_ps, d2.ts - d1.ts + 1)
    dists = np.zeros(d2.ts - d1.ts + 1, dtype=float)
    tm_sum = 0.0
    prev_v = 0.0
    for i, v in enumerate(vels_ps):
        tm_sum += (v + prev_v) / 2
        prev_v = v
        dists[i] = tm_sum

    return dists, vels_ps


def variable_speed_func(d1, d2):
    dists, vels_base = gradual_speed_func(d1, d2)
    S_tar = np.linalg.norm(d2.pos - d1.pos)
    # S_cur = np.sum(dists)
    S_cur = dists[-1]
    S_dif = S_tar - S_cur
    tm_interval = d2.ts - d1.ts

    sp_top = 2 * S_dif / tm_interval

    vels_ps1 = np.linspace(0.0, sp_top, tm_interval - (tm_interval // 2) + 1)
    vels_ps2 = np.linspace(sp_top, 0.0, tm_interval // 2 + 1)
    vels_ps = np.append(vels_ps1, vels_ps2[1:])
    vels_ps = vels_ps + vels_base
    ad_dists = np.zeros(tm_interval + 1, dtype=float)
    tm_sum = 0.0
    prev_v = 0.0
    for i, v in enumerate(vels_ps):
        tm_sum += (v + prev_v) / 2
        prev_v = v
        ad_dists[i] = tm_sum

    mod_dists = dists + ad_dists

    return mod_dists, vels_ps


@njit
def roots2(a, b, c):
    D = (b*b - 4*a*c)**0.5
    if D >= 0:
        return True, (-b + D)/(2*a), (-b - D)/(2*a)
    else:
        return False, 0.0, 0.0


@njit
def ellipse_point_inside(a, b, x, y):
    val = ((x*x)/(a*a)) + ((y*y)/(b*b))
    return val <= 1, val**0.5


@njit
def ellipse_line_intersect(a, b, k, c):
    ra = (k*k*a*a)+b*b
    rb = 2*k*c*(a*a)
    rc = ((c*c)-(b*b))*(a*a)
    inter, r1, r2 = roots2(ra, rb, rc)
    if not inter:
        return False, np.array([0.0, 0.0]), np.array([0.0, 0.0])
    else:
        return True, np.array([r1, k*r1+c]), np.array([r2, k*r2+c])


@njit
def line_cof(x1, y1, x2, y2):
    fi = 0.0000001

    k = (y2 - y1) / (x2 - x1 + fi)
    b = y1 - k * x1
    k = k if k != 0 else fi
    return k, b


@njit
def intersect(k1, b1, k2, b2):
    if round(k1, 4) == round(k2, 4):
        return False, 0, 0

    x = (b2 - b1) / (k1 - k2)
    y = k1 * x + b1
    return True, x, y


@njit
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


def sect_intersect2(p11, p12, p21, p22):
    return sect_intersect(p11[0], p11[1], p12[0], p12[1], p21[0], p21[1], p22[0], p22[1])


def sect_triangle_intersect(p1, p2, t, k=0.0, b=0.0):
    for i in range(3):
        incl, x, y = sect_intersect2(p1, p2, t[i-1], t[i])
        if incl:
            p_incl = np.array([x, y])
            p_end = p2 - p_incl
            dr = strech_vector_to(p_end, k * np.linalg.norm(p_end) + b)
            p = p_incl + dr
            return incl, p[0], p[1]

    return False, 0, 0


def dist(x1, y1, x2, y2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def ts_to_t(ts):
    return STEP * ts


def map_ranges(rng, st0, fi0, st1, fi1, default=0.0):
    if fi0 - st0 == 0.0:
        def_rng = np.zeros(len(rng), dtype=float)
        def_rng.fill(default)
        return def_rng

    mul = (fi1 - st1) / (fi0 - st0)
    rng_arr = np.array(rng)
    rng_arr = rng_arr - st0
    rng_arr = rng_arr * mul
    rng_arr = rng_arr + st1

    return rng_arr


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
            # if X[index] - X[j] == 0:
            # a = 0
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


def hermite2(Y, dY, t):
    X = [t[0], t[-1]]
    L1 = lagrange_single(X, 0, t)
    dL1 = d_lagrange_single(X, 0)
    U1 = (1 - 2 * dL1 * (t - X[0])) * L1 ** 2
    V1 = (t - X[0]) * L1 ** 2
    L2 = lagrange_single(X, 1, t)
    dL2 = d_lagrange_single(X, 1)
    U2 = (1 - 2 * dL2 * (t - X[1])) * L2 ** 2
    V2 = (t - X[1]) * L2 ** 2

    interp_values = U1 * Y[0] + V1 * dY[0] + U2 * Y[1] + V2 * dY[1]
    return interp_values


def connect_splines(Y, dY, t):
    visible_y = []
    for i in range(len(Y) - 1):
        Y_pair = Y[i:i + 2]
        dY_pair = dY[i:i + 2]
        spline_interval = hermite2(Y_pair, dY_pair, t[i])
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


def connect_times(dstl, func):
    continious = []
    vel_ps = []
    chunked = []
    base = 0.0
    for i in range(len(dstl) - 1):
        dist_steps, vels = func(dstl[i], dstl[i + 1])
        dist_steps = dist_steps + base
        base = dist_steps[-1]

        time_steps = map_ranges(dist_steps, dist_steps[0], dist_steps[-1], ts_to_t(
            dstl[i].ts), ts_to_t(dstl[i + 1].ts))

        # time_steps = np.linspace(ts_to_t(dstl[i].ts), ts_to_t(
        # dstl[i + 1].ts), dstl[i + 1].ts - dstl[i].ts)

        chunked.append(time_steps)
        continious.extend(time_steps[1:])
        vel_ps.extend(vels[1:])
        # if len(continious) == 0:
        # continious.extend(time_steps)
        # else:
        # continious.extend(time_steps[1:])
    return np.array(continious), chunked, np.array(vel_ps)

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


@njit
def get_vectors_cosine(a, b):
    la = np.linalg.norm(a)
    lb = np.linalg.norm(b)
    if la == 0 or lb == 0:
        return 0.0
    else:
        return get_dot_product(a, b) / (la * lb)


@njit
def get_vectors_sine(a, b):
    la = np.linalg.norm(a)
    lb = np.linalg.norm(b)
    if la == 0 or lb == 0:
        return 0.0
    else:
        return np.linalg.norm(get_cross_product(a, b)) / (la * lb)


@njit
def get_vectors_angle(a, b):
    return math.acos(get_vectors_cosine(a, b))


@njit
def get_cross_product(a, b):
    # return np.cross(a, b)
    return np.array([a[1] * b[2] - a[2] * b[1],  a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


@njit
def get_dot_product(a, b):
    return np.sum(a * b, axis=0)


@njit
def get_mv_dot_product(mat, vec):
    return np.sum(mat * vec, axis=1)


@njit
def identity(n):
    arr = np.zeros((n, n), dtype=float)
    for i in range(n):
        arr[i, i] = 1.0

    return arr


@njit
def get_rotation_matrix_from_two_vectors(a, b):
    a = strech_vector_to(a, 1)
    b = strech_vector_to(b, 1)
    phi = math.acos(get_vectors_cosine(a, b))
    axis = strech_vector_to(get_cross_product(a, b), 1)
    # matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    matrix = identity(3)
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


@njit
def get_4x4_from_3x3_mat(arr: np.ndarray):
    n_arr = identity(4)
    n_arr[:3, :3] = arr
    return n_arr


def get_3_from_2_arr(arr: np.ndarray):
    new_arr = np.zeros(3)
    new_arr[:2] = arr[:2]
    return new_arr


@njit
def get_2x2_rotation_matrix_from_angle(phi):
    matrix = np.zeros((2, 2), dtype=float)

    alpha_sin = math.sin(phi)
    alpha_cos = math.cos(phi)

    matrix[0, 0] = alpha_cos
    matrix[0, 1] = -alpha_sin
    matrix[1, 0] = alpha_sin
    matrix[1, 1] = alpha_cos

    return matrix


@njit
def strech_vector_to(v: np.ndarray, lenght):
    lv = np.linalg.norm(v)
    if lv != 0:
        return v * (lenght / lv)
    else:
        return np.zeros(v.shape, dtype=float)
        # return np.array([0.0, 0.0, 0.0])


@njit
def point_line_projection(pt, k, b):
    _, x, y = intersect(k, b, -(1/k),  pt[1] + pt[0]/k)
    pt2 = np.array([x, y])
    return pt2


@njit
def point_to_line(pt, k, b) -> float:
    pt2 = point_line_projection(pt, k, b)
    dist = np.linalg.norm(pt - pt2)
    return dist


@njit
def point_to_sect(pt, pta, ptb):
    k, b = line_cof(pta[0], pta[1], ptb[0], ptb[1])
    if k == 0:
        k += 0.000001  # some small value
    ptc = point_line_projection(pt, k, b)
    incl = round(min(pta[0], ptb[0]), 3) <= round(ptc[0], 3) <= round(max(pta[0], ptb[0]), 3) and round(
        min(pta[1], ptb[1]), 3) <= round(ptc[1], 3) <= round(max(pta[1], ptb[1]), 3)
    if not incl:
        return -1.0

    dist = point_to_line(pt, k, b)
    return dist


@njit
def point_to_line2(pt, pta, ptb):
    k, b = line_cof(pta[0], pta[1], ptb[0], ptb[1])
    if k == 0:
        k += 0.000001  # some small value
    dist = point_to_line(pt, k, b)
    return dist


@njit
def vector_projection(from_vec, to_vec):
    return (get_dot_product(from_vec, to_vec)) / np.linalg.norm(to_vec)


@njit
def area(p1, p2, p3):
    return abs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])) / 2.0)


@njit
def is_inside(p, a, b, c):
    A = area(a, b, c)
    A1 = area(p, b, c)
    A2 = area(p, a, c)
    A3 = area(p, a, b)

    return (round(A, 5) == round(A1 + A2 + A3, 5))


@njit
def is_inside2(p, a, b, c, r):
    if not is_inside(p, a, b, c):
        return False

    return point_to_line2(p, a, b) >= r and \
        point_to_line2(p, b, c) >= r and \
        point_to_line2(p, a, c) >= r


@njit
def xor(a, b):
    return bool((a and not b) or (not a and b))


def centroid_of_polygon(pts):
    crs_sum = 0.0
    x_sum = 0.0
    y_sum = 0.0

    for i in range(len(pts)):
        j = (i + 1) % len(pts)
        cross = pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
        crs_sum += cross
        x_sum += (pts[i][0] + pts[j][0]) * cross
        y_sum += (pts[i][1] + pts[j][1]) * cross

    z = 1.0 / (3.0 * crs_sum)
    return z * np.array([x_sum, y_sum])


def make_cw(arr):
    cw_seq = [0, 1, 3, 2]
    ans = np.array([arr[cw_seq[i]] for i in range(4)])
    return ans
