import matplotlib.pyplot as plt
import numpy as np

from interp import *
import pybullet as p


class DestPoint:
    def __init__(self, pos, t, dx=None, dy=None, dz=None):
        self.t = t
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.dx = dx
        self.dy = dy
        self.dz = dz

    @classmethod
    def default(cls):
        return cls([0.0, 0.0, 0.0], 0.0)

    def clone(self):
        return DestPoint([self.x, self.y, self.z], self.t, self.dx, self.dy, self.dz)


class Logger:
    def __init__(self):
        self.targets = []
        self.points = []
        self.steps = []


class Plan:
    def __init__(self, leg, start: DestPoint = None):
        start_ = DestPoint(leg.position, 0.0)

        self.leg = leg
        self.need_plan: bool = True
        self.cur: DestPoint = start_
        self.last: DestPoint = start_
        self.points: list = []
        self.steps: list = []
        self.target: DestPoint = DestPoint.default()
        self.log: Logger = Logger()
        self.obstacles: list = []
        self.adj: np.ndarray = np.zeros(3, np.float)

        self.log.targets.append(start_)

    def reset(self):
        self.points.clear()
        self.steps.clear()
        self.need_plan = True

    def adjust(self, x=0.0, y=0.0, z=0.0):
        self.adj = self.adj + np.array([x, y, z])
        # print(f"pos:{self.leg.position[2]} adjust:{z} total:{self.adj[2]}")


    def plan_steps(self):
        time = np.array([i.t for i in self.points])
        data_x = np.array([i.x for i in self.points])
        data_y = np.array([i.y for i in self.points])
        data_z = np.array([i.z for i in self.points])
        data_dx = np.array([i.dx for i in self.points])
        data_dy = np.array([i.dy for i in self.points])
        data_dz = np.array([i.dz for i in self.points])

        # self.log.points.extend(self.points)

        spline_x = connect_splines2(time, data_x, data_dx, time)
        spline_y = connect_splines2(time, data_y, data_dy, time)
        spline_z = connect_splines2(time, data_z, data_dz, time)
        time_steps = connect_times(time)
        dx = akima(np.array(time_steps), np.array(spline_x))
        dy = akima(np.array(time_steps), np.array(spline_y))
        dz = akima(np.array(time_steps), np.array(spline_z))

        steps = [DestPoint([x, y, z], t, dx_, dy_, dz_) for x, y, z, t, dx_, dy_, dz_ in zip(spline_x, spline_y, spline_z, time_steps, dx, dy, dz)]
        self.steps = steps

    def step(self):
        self.last = self.cur
        s = self.steps[0]
        self.cur = s
        self.leg.position = np.array([s.x, s.y, s.z]) + self.adj
        # do something

        # self.log.steps.append(s)
        self.steps.pop(0)

    def step_zero(self):
        self.leg.position = np.array([self.cur.x, self.cur.y, self.cur.z]) + self.adj

    def check_damp(self):
        l_idx = self.leg.idx
        l_tf = self.leg.body.sens_info.touch_force[l_idx]

        damp_val = p.getJointState(self.leg.body.model, self.leg.dampener)[0]

        damp_val_n = damp_val / T_RAD
        # u_thr = 0.7 * T_RAD
        # l_thr = 0.3 * T_RAD

        dst = 0.0
        if damp_val_n < 0.85:
            dst = 0.5 * (damp_val - T_RAD)

        return bool(l_tf), dst

    def plot(self):
        plt.figure()
        ax = plt.axes()
        ax.set_aspect('equal')
        # for b in self.boxes:
        for b in self.obstacles:
            plt.plot([b.x1, b.x2], [b.y1, b.y2], color="black")

        pts_x = [i.x for i in self.log.points]
        pts_y = [i.y for i in self.log.points]
        pts_t = [i.t for i in self.log.points]

        tar_x = [i.x for i in self.log.targets]
        tar_y = [i.y for i in self.log.targets]
        tar_t = [i.t for i in self.log.targets]

        st_x = [i.x for i in self.log.steps]
        st_y = [i.y for i in self.log.steps]
        st_t = [i.t for i in self.log.steps]

        plt.plot(st_x, st_y, color="blue")
        plt.scatter(pts_x, pts_y, color="red")
        plt.scatter(tar_x, tar_y, color="green")
        plt.grid("both")
        plt.show()
