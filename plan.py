import matplotlib.pyplot as plt
from interp import *


class DestPoint:
    def __init__(self, x, y, t, dx=None, dy=None):
        self.t = t
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy

    @classmethod
    def default(cls):
        return cls(0, 0, 0)

    def clone(self):
        return DestPoint(self.x, self.y, self.t, self.dx, self.dy)


class Logger:
    def __init__(self):
        self.targets = []
        self.points = []
        self.steps = []


class Plan:
    def __init__(self, leg, start: DestPoint = None):
        if not start:
            start_ = DestPoint(0, 0, 0)
        else:
            start_ = start

        self.leg = leg
        self.need_plan: bool = True
        self.cur: DestPoint = start_
        self.last: DestPoint = start_
        self.points: list = []
        self.steps: list = []
        # self.steps = [start_]
        self.target: DestPoint = DestPoint.default()
        self.log: Logger = Logger()
        self.obstacles: list = []

        self.log.targets.append(start_)

    def reset(self):
        self.points.clear()
        self.steps.clear()
        self.need_plan = True

    def adjust(self, x, y):
        for s in self.steps:
            s.x += x
            s.y += y

        self.target.x += x
        self.target.y += y

    def plan_steps(self):
        time = np.array([i.t for i in self.points])
        data_x = np.array([i.x for i in self.points])
        data_y = np.array([i.y for i in self.points])
        data_dx = np.array([i.dx for i in self.points])
        data_dy = np.array([i.dy for i in self.points])

        self.log.points.extend(self.points)

        spline_x = connect_splines2(time, data_x, data_dx, time)
        spline_y = connect_splines2(time, data_y, data_dy, time)
        time_steps = connect_times(time)
        dx = akima(np.array(time_steps), np.array(spline_x))
        dy = akima(np.array(time_steps), np.array(spline_y))

        steps = [DestPoint(x, y, t, dx_, dy_) for x, y, t, dx_, dy_ in zip(spline_x, spline_y, time_steps, dx, dy)]
        self.steps = steps

    def step(self):
        self.last = self.cur
        s = self.steps[0]
        self.cur = s
        # do something

        self.log.steps.append(s)
        self.steps.pop(0)

    def check2(self):
        hits = False
        dst = float("-inf")
        for o in self.obstacles:
            if o.perp:
                continue

            d = o.hits(self.cur) - self.cur.y

            if o.x1 <= self.cur.x <= o.x2 and 0 <= abs(d) <= T_RAD and d > dst:
                dst = d
                hits = True

        return hits, dst if hits else 0

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
