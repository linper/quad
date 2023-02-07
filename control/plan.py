import matplotlib.pyplot as plt
import numpy as np

from interp import *
from math import floor
import pybullet as p


def get_path_len(start_pt, pts) -> float:
    pos_lst = [start_pt.pos]
    pos_lst.extend([p.pos for p in pts])
    pos_difs = [np.linalg.norm(p1 - p0)
                for p0, p1 in zip(pos_lst[:-1], pos_lst[1:])]
    S = np.sum(np.array(pos_difs))
    return S


def fill_diffs(lst):
    time_steps = np.array([float(ts_to_t(i.ts)) for i in lst])
    data_x = np.array([float(i.pos[0]) for i in lst])
    data_y = np.array([float(i.pos[1]) for i in lst])
    data_z = np.array([float(i.pos[2]) for i in lst])

    dx = akima(time_steps, data_x)
    dy = akima(time_steps, data_y)
    dz = akima(time_steps, data_z)

    for i in range(len(lst)):
        vel = np.copy(lst[i].vel)
        if np.isnan(lst[i].vel[0]):
            vel[0] = dx[i]
            # lst[i].vel[0] = dx[i]

        if np.isnan(lst[i].vel[1]):
            vel[1] = dy[i]
            # lst[i].vel[1] = dy[i]

        if np.isnan(lst[i].vel[2]):
            vel[2] = dz[i]
            # lst[i].vel[2] = dz[i]

        lst[i].set_vel(vel)

def line_pts_intersect_r2(p1, p2, pts, i1, r):
    k, b = line_cof(p1[0], p1[1], p2[0], p2[1])
    # print(f"k:{k} b:{b}")
    i_max = len(pts) - 1
    i2 = i1
    found = False
    s1 = s2 = k * pts[i1][0] + b > pts[i1][1]

    # Find i2
    while i2 < i_max:
        i2 += 1
        s2 = k * pts[i2][0] + b > pts[i2][1]
        # print(f"i:{i2} {s1}:{s2} {round(pts[i2][0], 5)} {round((k * pts[i2][0] + b) - pts[i2][1], 4)}")
        if xor(s1, s2):
            found = True
            break

    if not found:
        return -1, -1
        
    # Select closer point as mid
    s1_dist = abs(k * pts[i1][0] + b - pts[i1][1])
    s2_dist = abs(k * pts[i2][0] + b - pts[i2][1])

    i_lo = i_hi = i_mid = i1 if s1_dist < s2_dist else i2
    while i_lo >= 0 and np.linalg.norm(pts[i_mid] - pts[i_lo]) < r:
        i_lo -= 1

    while i_hi < len(pts) and np.linalg.norm(pts[i_mid] - pts[i_hi]) < r:
        i_hi += 1

    return i_lo, i_hi


def line_pts_intersect_r(p1, p2, pts, i1, i2, r):
    k, b = line_cof(p1[0], p1[1], p2[0], p2[1])

    # Binnary search
    s1 = k * pts[i1][0] + b > pts[i1][1]
    s2 = k * pts[i2][0] + b > pts[i2][1]
    print(f"k:{k} b:{b} i1:{i1} i2:{i2}")
    print(f"x:{pts[i1][0]} yc:{pts[i1][1]} yt:{k * pts[i1][0] + b}")
    print(f"x:{pts[i2][0]} yc:{pts[i2][1]} yt:{k * pts[i2][0] + b}")
    if not xor(s1, s2):
        print("Not intersecting")
        return False, None, None

    print(f"i1:{i1} i2:{i2}")
    while abs(i1 - i2) > 1:
        i_mid = (i2 + i1) >> 1
        s_mid = k * pts[i_mid][0] + b > pts[i_mid][1]
        if xor(s1, s_mid):
            print(f"i1:{i1}")
            i1 = i_mid
        else:
            print(f"i2:{i2}")
            i2 = i_mid

    # Select closer point as mid
    s1_dist = abs(k * pts[i1][0] + b - pts[i1][1])
    s2_dist = abs(k * pts[i2][0] + b - pts[i2][1])

    i_lo = i_hi = i_mid = i1 if s1_dist < s2_dist else i2
    while i_lo > i1 and np.linalg.norm(pts[i_mid] - pts[i_lo]) < r:
        i_lo -= 1

    while i_hi < i2 and np.linalg.norm(pts[i_mid] - pts[i_hi]) < r:
        i_hi -= 1

    return True, pts[i_lo], pts[i_hi]


def prepare_pending_points(start_pt):
    res_pts= [DestPoint(np.copy(start_pt.pos), f_arr_unset(),
                         ts=0, vel_ps=0.0)]
    return res_pts


def prepare_ascending_points(pt, n_steps):
    print(f"ASC PT:{pt} n_steps:{n_steps}")
    res_pts = [DestPoint(pt, f_arr_unset(), ts=n_steps)]
    return res_pts


def prepare_traversing_points(start_pt, pts, speed_ps: float):
    # Getting positions of every path point
    pos_lst= [start_pt.pos]
    pos_lst.extend([p.pos for p in pts])

    # Getting lengths of every path interval
    pos_difs= [0.0] + [np.linalg.norm(p1 - p0)
                        for p0, p1 in zip(pos_lst[:-1], pos_lst[1:])]
    # S = np.sum(np.array(pos_difs))

    # Aggregating lengths of every path point
    pos_dif_agr= []
    pos_dif_agr_last= 0.0
    for pd in pos_difs:
        pos_dif_agr.append(pos_dif_agr_last + pd)
        pos_dif_agr_last= pos_dif_agr[-1]

    # Getting velocities of every path point
    vels = map_ranges(
        pos_dif_agr, pos_dif_agr[0], pos_dif_agr[-1], start_pt.vel_ps, speed_ps, default=speed_ps)
    # Getting number of steps of every path point
    n_steps_unrounded = [
        2 * s / (v0 + v1) for s, v0, v1 in zip(pos_difs[1:], vels[:-1], vels[1:])]

    # n_steps_unrounded = [2 * s / (v0 + v1) for s, v0, v1 in zip(pos_difs, np.array([start_pt.vel_ps] + list(vels[:-1])), vels) ]
    # Aggregating number of steps of every path point
    n_steps_agr = [0]
    n_steps_agr_last = n_steps_agr[-1]
    for s in n_steps_unrounded:
        n_steps_agr.append(floor(n_steps_agr_last + s))
        n_steps_agr_last = n_steps_agr[-1]

    # Getting DestPoint of every path point
    res_pts = [DestPoint(p, f_arr_unset(), ts=s, vel_ps=vps)
               for p, s, vps in zip(pos_lst, n_steps_agr, vels)]
               # for p, s, vps in zip(pos_lst[1:], n_steps_agr, vels[1:])]

    return res_pts


def plan_steps(points, speed_func):
    time_points = np.array([i.ts for i in points])
    data_x = np.array([i.pos[0] for i in points])
    data_y = np.array([i.pos[1] for i in points])
    data_z = np.array([i.pos[2] for i in points])
    data_dx = np.array([i.vel[0] for i in points])
    data_dy = np.array([i.vel[1] for i in points])
    data_dz = np.array([i.vel[2] for i in points])

    ts_connected = np.array(
        list(range(time_points[0] + 1, time_points[-1] + 1)))
    time_steps, time_steps_chunked, vels_ps = connect_times(
        points, speed_func)

    spline_x = connect_splines(data_x, data_dx, time_steps_chunked)
    spline_y = connect_splines(data_y, data_dy, time_steps_chunked)
    spline_z = connect_splines(data_z, data_dz, time_steps_chunked)

    dx = akima(np.array(time_steps), np.array(spline_x))
    dy = akima(np.array(time_steps), np.array(spline_y))
    dz = akima(np.array(time_steps), np.array(spline_z))

    steps = [DestPoint(np.array([x, y, z]), np.array([dx_, dy_, dz_]), t, ts=ts, vel_ps=v_ps) for x, y, z, t, ts, dx_, dy_, dz_, v_ps in zip(
        spline_x, spline_y, spline_z, time_steps, ts_connected, dx, dy, dz, vels_ps)]
    steps.reverse()
    # print(f"steps len:{len(steps)}")
    return steps


class DestPoint:
    def __init__(self, pos: np.ndarray, vel: np.ndarray, t: float = -1.0, ts: int = -1, vel_ps: float = 0.0):
        self.t: float = t
        self.ts: int = ts
        self.pos: np.ndarray = pos
        self.vel: np.ndarray = vel
        self.vel_ps: float = vel_ps
        self.x: float = pos[0]
        self.y: float = pos[1]
        self.z: float = pos[2]
        self.dx: float = vel[0]
        self.dy: float = vel[1]
        self.dz: float = vel[2]

    @classmethod
    def default(cls):
        return cls(np.zeros(3, dtype=float), np.zeros(3, dtype=float), 0.0)

    @staticmethod
    def dest_pts_lst(lst):
        res = []
        for l in lst:
            if len(l) == 2:
                l = np.array([l[0], l[1], 0.0])
            res.append(DestPoint(l, np.zeros(3, dtype=float), 0.0))

        return res

    def clone(self):
        return DestPoint(np.copy(self.pos), np.copy(self.vel), self.t, self.ts, self.vel_ps)

    def set_pos(self, pos: np.ndarray):
        self.pos = pos
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]

    def set_vel(self, vel: np.ndarray):
        self.vel = vel
        self.dx = vel[0]
        self.dy = vel[1]
        self.dz = vel[2]


class Logger:
    def __init__(self):
        self.targets = []
        self.points = []
        self.steps = []


class PlanView:
    def __init__(self, p):
        self.need_plan: bool = p.need_plan
        self.cur: DestPoint = p.cur.clone()
        self.target: DestPoint = p.target.clone()
        self.adj: np.ndarray = p.adj.copy()

        # self.log.targets.append(start_)


class Plan:
    def __init__(self, leg):
        start_ = DestPoint(np.copy(leg.position), f_arr_unset(), 0.0)

        self.leg = leg
        self.need_plan: bool = True
        self.cur: DestPoint = start_
        # self.last: DestPoint = start_
        self.raw_points: list = []
        self.points: list = []
        self.steps: list = []
        # self.steps: np.ndarray
        # self.est_n_steps: int = 0
        self.speed_func = None
        self.target: DestPoint = DestPoint(
            leg.position, np.zeros(3, dtype=float), 0.0)
        self.log: Logger = Logger()
        self.adj: np.ndarray = np.zeros(3, np.float)

        self.log.targets.append(start_)

    def get_view(self):
        return PlanView(self)

    def reset(self):
        self.raw_points.clear()
        self.points.clear()
        self.steps.clear()
        # self.step_idx = 0
        self.need_plan = True
        # self.est_n_steps = 0
        self.speed_func = None

    def adjust(self, adj: np.ndarray):
        new_adj = self.adj + adj

        if new_adj[2] + self.cur.pos[2] < MAX_DIP:
            new_adj[2] = MAX_DIP - self.cur.pos[2]

        if new_adj[2] + self.cur.pos[2] > MIN_DIP:
            new_adj[2] = MIN_DIP - self.cur.pos[2]

        self.adj = new_adj
        # print(f"pos:{self.leg.position[2]} adjust:{z} total:{self.adj[2]}")

    def compensate_pos(self):
        self.cur.pos = self.cur.pos + self.adj
        self.adj = np.zeros(3, dtype=float)

    def plan_steps(self):
        self.steps = plan_steps(self.points, self.speed_func)
        return

        time_points = np.array([i.ts for i in self.points])
        data_x = np.array([i.pos[0] for i in self.points])
        data_y = np.array([i.pos[1] for i in self.points])
        data_z = np.array([i.pos[2] for i in self.points])
        data_dx = np.array([i.vel[0] for i in self.points])
        data_dy = np.array([i.vel[1] for i in self.points])
        data_dz = np.array([i.vel[2] for i in self.points])

        ts_connected = np.array(
            list(range(time_points[0] + 1, time_points[-1] + 1)))
        time_steps, time_steps_chunked, vels_ps = connect_times(
            self.points, self.speed_func)

        spline_x = connect_splines(data_x, data_dx, time_steps_chunked)
        spline_y = connect_splines(data_y, data_dy, time_steps_chunked)
        spline_z = connect_splines(data_z, data_dz, time_steps_chunked)

        dx = akima(np.array(time_steps), np.array(spline_x))
        dy = akima(np.array(time_steps), np.array(spline_y))
        dz = akima(np.array(time_steps), np.array(spline_z))

        steps = [DestPoint(np.array([x, y, z]), np.array([dx_, dy_, dz_]), t, ts=ts, vel_ps=v_ps) for x, y, z, t, ts, dx_, dy_, dz_, v_ps in zip(
            spline_x, spline_y, spline_z, time_steps, ts_connected, dx, dy, dz, vels_ps)]
        steps.reverse()
        print(f"steps len:{len(steps)}")
        self.steps = steps

    def step(self):
        self.cur = self.steps.pop()
        self.leg.position = self.cur.pos + self.adj

    def step_zero(self):
        self.leg.position = self.cur.pos + self.adj
