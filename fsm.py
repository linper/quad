from plan import DestPoint
from balance import get_balance, get_walk_height
from plan import *
import functools
from enum import IntEnum
from math import floor
from interp import variable_speed_func

import cProfile

min_damp = 0.0
max_damp = 0.0

misc_data = []


class FSMState(IntEnum):
    STOPPED = 0
    ASCENDING = 1
    DESCENDING = 2
    TRAVERSING = 3
    PENDING = 4
    MAX__ = 5


class FSMAction(IntEnum):
    NO_ACT = 0
    END = 1
    HIT = 2
    STOP = 3
    MAX__ = 4


class FSMInfo():
    def __init__(self):
        self.step_h: float = 0.0

    def clear(self):
        self.step_h = 0.0

    def clone(self):
        cl = FSMInfo()
        cl.step_h = self.step_h
        return cl


class FSMView:
    def __init__(self, fsm):
        self.cur: FSMState = fsm.cur
        self.info: FSMInfo = fsm.info.clone()


class FSM:
    def __init__(self, leg):
        self.leg = leg
        self.cur: FSMState = FSMState.STOPPED
        self.info: FSMInfo = FSMInfo()
        self.func: dict = {
            FSMState.STOPPED: self.act_stopped,
            FSMState.ASCENDING: self.act_ascending,
            FSMState.DESCENDING: self.act_descending,
            FSMState.TRAVERSING: self.act_traversing,
            FSMState.PENDING: self.act_pending, }

        self.map: np.ndarray = np.zeros((FSMAction.MAX__, FSMState.MAX__), int)

        self.map[FSMAction.NO_ACT][FSMState.STOPPED] = FSMState.STOPPED
        self.map[FSMAction.STOP][FSMState.STOPPED] = FSMState.STOPPED
        self.map[FSMAction.END][FSMState.STOPPED] = FSMState.PENDING

        self.map[FSMAction.NO_ACT][FSMState.ASCENDING] = FSMState.ASCENDING
        self.map[FSMAction.END][FSMState.ASCENDING] = FSMState.DESCENDING
        self.map[FSMAction.STOP][FSMState.ASCENDING] = FSMState.STOPPED

        self.map[FSMAction.NO_ACT][FSMState.DESCENDING] = FSMState.DESCENDING
        self.map[FSMAction.END][FSMState.DESCENDING] = FSMState.PENDING
        self.map[FSMAction.HIT][FSMState.DESCENDING] = FSMState.PENDING
        self.map[FSMAction.STOP][FSMState.DESCENDING] = FSMState.STOPPED

        self.map[FSMAction.NO_ACT][FSMState.TRAVERSING] = FSMState.TRAVERSING
        self.map[FSMAction.END][FSMState.TRAVERSING] = FSMState.PENDING
        self.map[FSMAction.STOP][FSMState.TRAVERSING] = FSMState.STOPPED

        self.map[FSMAction.NO_ACT][FSMState.PENDING] = FSMState.PENDING
        self.map[FSMAction.STOP][FSMState.PENDING] = FSMState.STOPPED

    def get_view(self):
        return FSMView(self)

    @classmethod
    def state_str(cls, fsm):
        str_dict = {
            FSMState.STOPPED: "STP",
            FSMState.ASCENDING: "ASC",
            FSMState.DESCENDING: "DES",
            FSMState.TRAVERSING: "TRV",
            FSMState.PENDING: "PND"}
        return str_dict[fsm.cur]

    def next(self, act: FSMAction):
        st = self.map[act][self.cur]
        self.cur = st
        self.leg.plan.compensate_pos()
        self.execute()

    def set(self, state: FSMState):
        self.cur = state
        self.leg.plan.compensate_pos()

    def reset(self):
        self.info.clear
        self.cur = FSMState.STOPPED

    def execute(self):
        # self.leg.plan.log.targets.append(self.leg.plan.target)
        self.func[self.cur]()

    def plot(self, arr):
        plt.figure()
        time_data = list(range(len(misc_data)))

        plt.plot(time_data, misc_data)
        plt.grid("both")
        plt.show()

    def act_ascending(self):
        TOP_TM_C = 0.4
        MID_PT_HT_C = 0.75
        END_OF_C = 0.5
        MID_OF_C = 0.25
        p = self.leg.plan

        if p.need_plan:
            print(f"{self.leg.name} : Ascending")
            self.leg.do_balance = False
            start = DestPoint(np.copy(p.cur.pos), f_arr_unset(), ts=0)
            end_ts = int(floor(TOP_TM_C * (p.target.ts - start.ts)))
            step_dist = np.linalg.norm(
                np.array([p.target.x - start.x, p.target.y - start.y]))
            self.info.walk_h = get_walk_height(
                step_dist, self.leg.body.sens_info.abs_std_leg_h)

            end = DestPoint(
                np.array([(start.x + END_OF_C * (p.target.x - start.x)),
                          (start.y + END_OF_C * (p.target.y - start.y)),
                          self.info.walk_h]),
                np.array([F_UNSET, F_UNSET, 0.0]), ts=end_ts)

            mid = DestPoint(
                np.array([start.x + MID_OF_C * (p.target.x - start.x),
                          start.y + MID_OF_C * (p.target.y - start.y),
                          start.z + MID_PT_HT_C * (self.info.walk_h - start.z)]), f_arr_unset(),
                ts=int(end_ts // 2))

            start.vel_ps = 0.0
            mid.vel_ps = get_d2_speed(start, mid)
            end.vel_ps = get_d2_speed(mid, end)
            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(mid)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        if len(p.steps) > 0:
        # if p.step_idx < p.steps.size:
            # global misc_data
            # misc_data.append(p.steps[0].z)

            p.step()
        else:
            walk_h = self.info.walk_h
            p.reset()
            self.info.walk_h = walk_h

            # self.plot()
            # self.next(FSMAction.STOP)
            self.next(FSMAction.END)

    def act_descending(self):
        PRE_LAND_PT_HT_C = 0.2
        PRE_LAND_PT_OF_C = 0.96
        PRE_LAND_PT_TM_C = 0.7
        p = self.leg.plan

        if p.need_plan:
            print(f"{self.leg.name} : Descending")
            self.leg.do_balance = False
            start = p.cur.clone()
            end = DestPoint(
                np.copy(p.target.pos), np.array([0, F_UNSET, F_UNSET]), ts=p.target.ts)

            # self.leg.plan.log.points.append(end)

            mid_ts = round(start.ts + PRE_LAND_PT_TM_C *
                           (p.target.ts - start.ts))
            mid = DestPoint(
                np.array([start.x + PRE_LAND_PT_OF_C * (p.target.x - start.x),
                          start.y + PRE_LAND_PT_OF_C * (p.target.y - start.y),
                          end.z + PRE_LAND_PT_HT_C * (self.info.walk_h - end.z)]),
                f_arr_unset(),
                ts=mid_ts)

            mid.vel_ps = get_d2_speed(start, mid)

            p.points.append(start)
            p.points.append(mid)
            p.points.append(end)

            self.leg.plan.speed_func = variable_speed_func

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        hits, s_hits, _ = self.leg.check_damp()

        if hits:
            print(f"{p.leg.name} hit")
        if s_hits:
            print(
                f"{p.leg.name} soft hit:{self.leg.body.sens_info.damp[self.leg.idx]}")

        if hits:
            p.reset()
            self.next(FSMAction.HIT)
        elif len(p.steps) > 0:
        # elif p.step_idx < p.steps.size:
            p.step()
        else:
            p.reset()
            self.next(FSMAction.END)

    def act_traversing(self):
        p = self.leg.plan

        if p.need_plan:
            print(f"{self.leg.name} : Traversing")
            self.leg.do_balance = True
            start = DestPoint(np.copy(p.cur.pos),
                              np.array([0.0, 0.0, 0.0]), ts=0, vel_ps=p.cur.vel_ps)

            # start = p.cur.clone()
            # start.ts = 0
            # start.dx = 0.0
            # start.dy = 0.0
            # start.dz = 0.0

            p.raw_points[-1].set_vel(np.array([0.0, 0.0, 0.0]))

            # end = DestPoint(np.copy(p.target.pos),
                            # np.array([0.0, 0.0, 0.0]), ts=p.target.ts, vel_ps=p.target.vel_ps)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.extend(p.raw_points)
            # p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            # if self.leg.name == "front_left":
                # plt.figure()
                # plt.plot(list(range(len(p.steps))), [s.vel_ps for s in p.steps], color="black")
                # plt.plot(list(range(len(p.steps))), [s.pos[0] for s in p.steps], color="red")
                # plt.plot(list(range(len(p.steps))), [s.pos[1] for s in p.steps], color="green")
                # plt.grid("both")
                # plt.show()

            p.need_plan = False

        if len(p.steps) == 0:
        # if p.step_idx == p.steps.size:
            p.reset()
            self.next(FSMAction.END)
        else:
            balance = get_balance(self.leg)
            p.adjust(balance)
            p.step()

    def act_stopped(self):
        self.leg.do_balance = False
        p = self.leg.plan

        if p.need_plan:
            print(f"{self.leg.name} : Stopped")
            p.need_plan = False

    def act_pending(self):
        p = self.leg.plan

        if p.need_plan:
            print(f"{self.leg.name} : Pending")
            p.need_plan = False
            self.leg.do_balance = True

        balance = get_balance(self.leg)
        p.adjust(balance)
        p.step_zero()
