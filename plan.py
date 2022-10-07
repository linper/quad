import matplotlib.pyplot as plt
import numpy as np

from interp import *
import pybullet as p


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

    def clone(self):
        return DestPoint(np.copy(self.pos), np.copy(self.vel), self.t, self.ts, self.vel_ps)


class Logger:
    def __init__(self):
        self.targets = []
        self.points = []
        self.steps = []


class Plan:
    def __init__(self, leg):
        start_ = DestPoint(np.copy(leg.position), f_arr_unset(), 0.0)

        self.leg = leg
        self.need_plan: bool = True
        self.cur: DestPoint = start_
        self.last: DestPoint = start_
        self.points: list = []
        self.steps: list = []
        self.est_n_steps: int = 0
        self.speed_func = None
        self.target: DestPoint = DestPoint(
            leg.position, np.zeros(3, dtype=float), 0.0)
        self.log: Logger = Logger()
        # self.obstacles: list = []
        self.adj: np.ndarray = np.zeros(3, np.float)

        self.log.targets.append(start_)

    def reset(self):
        self.points.clear()
        self.steps.clear()
        self.need_plan = True
        self.est_n_steps = 0
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
        time = np.array([i.t for i in self.points])
        data_x = np.array([i.pos[0] for i in self.points])
        data_y = np.array([i.pos[1] for i in self.points])
        data_z = np.array([i.pos[2] for i in self.points])
        data_dx = np.array([i.vel[0] for i in self.points])
        data_dy = np.array([i.vel[1] for i in self.points])
        data_dz = np.array([i.vel[2] for i in self.points])

        # self.log.points.extend(self.points)

        spline_x = connect_splines2(time, data_x, data_dx, time)
        spline_y = connect_splines2(time, data_y, data_dy, time)
        spline_z = connect_splines2(time, data_z, data_dz, time)
        time_steps = connect_times(time)
        dx = akima(np.array(time_steps), np.array(spline_x))
        dy = akima(np.array(time_steps), np.array(spline_y))
        dz = akima(np.array(time_steps), np.array(spline_z))

        steps = [DestPoint(np.array([x, y, z]), np.array([dx_, dy_, dz_]), t) for x, y, z, t, dx_, dy_, dz_ in zip(
            spline_x, spline_y, spline_z, time_steps, dx, dy, dz)]
        self.steps = steps

    def plan_steps2(self):
        time_points = np.array([i.ts for i in self.points])
        data_x = np.array([i.pos[0] for i in self.points])
        data_y = np.array([i.pos[1] for i in self.points])
        data_z = np.array([i.pos[2] for i in self.points])
        data_dx = np.array([i.vel[0] for i in self.points])
        data_dy = np.array([i.vel[1] for i in self.points])
        data_dz = np.array([i.vel[2] for i in self.points])

        # self.log.points.extend(self.points)

        ts_connected = np.array(
            list(range(time_points[0] + 1, time_points[-1] + 1)))
        time_steps, time_steps_chunked, vels_ps = connect_times2(
            self.points, self.speed_func)

        spline_x = connect_splines3(data_x, data_dx, time_steps_chunked)
        spline_y = connect_splines3(data_y, data_dy, time_steps_chunked)
        spline_z = connect_splines3(data_z, data_dz, time_steps_chunked)

        # time = np.array([ts_to_t(ts) for ts in time_points])
        # spline_x = connect_splines2(time, data_x, data_dx, time)
        # spline_y = connect_splines2(time, data_y, data_dy, time)
        # spline_z = connect_splines2(time, data_z, data_dz, time)
        # time_steps = connect_times(time)

        if self.leg.fsm.cur in [1, 2]:
            plt.figure()
            plt.plot(spline_x, spline_z, color="blue")
            # plt.plot(list(range(len(spline_x))), spline_x, color="blue")
            # plt.plot(list(range(len(spline_y))), spline_y, color="red")
            # plt.plot(list(range(len(spline_z))), spline_z, color="green")
            # plt.plot(list(range(len(time_steps))), time_steps, color="red")
            plt.grid("both")
            plt.show()

        dx = akima(np.array(time_steps), np.array(spline_x))
        dy = akima(np.array(time_steps), np.array(spline_y))
        dz = akima(np.array(time_steps), np.array(spline_z))

        # TODO: all lengths does not match  <04-10-22, yourname> #
        steps = [DestPoint(np.array([x, y, z]), np.array([dx_, dy_, dz_]), t, ts=ts, vel_ps=v_ps) for x, y, z, t, ts, dx_, dy_, dz_, v_ps in zip(
            spline_x, spline_y, spline_z, time_steps, ts_connected, dx, dy, dz, vels_ps)]
        self.steps = steps

    def step(self):
        self.last = self.cur
        self.cur = self.steps[0]
        self.leg.position = self.cur.pos + self.adj
        # print(f"step:{en(p.steps)}")
        # do something

        # self.log.steps.append(s)
        self.steps.pop(0)

    def step_zero(self):
        self.leg.position = self.cur.pos + self.adj

    def plot(self):
        plt.figure()
        ax = plt.axes()
        ax.set_aspect('equal')
        # for b in self.boxes:
        # for b in self.obstacles:
        # plt.plot([b.x1, b.x2], [b.y1, b.y2], color="black")

        pts_x = [i.pos[0] for i in self.log.points]
        pts_y = [i.pos[1] for i in self.log.points]
        pts_t = [i.pos[2] for i in self.log.points]

        tar_x = [i.pos[0] for i in self.log.targets]
        tar_y = [i.pos[1] for i in self.log.targets]
        tar_t = [i.pos[2] for i in self.log.targets]

        st_x = [i.pos[0] for i in self.log.steps]
        st_y = [i.pos[1] for i in self.log.steps]
        st_t = [i.pos[2] for i in self.log.steps]

        plt.plot(st_x, st_y, color="blue")
        plt.scatter(pts_x, pts_y, color="red")
        plt.scatter(tar_x, tar_y, color="green")
        plt.grid("both")
        plt.show()
