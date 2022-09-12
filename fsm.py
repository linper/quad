from enum import IntEnum
import functools

from plan import *
from balance import *
from plan import DestPoint

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


class FSM:
    def __init__(self, leg):
        self.leg = leg
        self.cur = FSMState.STOPPED
        self.func = {
            FSMState.STOPPED: self.act_stopped,
            FSMState.ASCENDING: self.act_ascending,
            FSMState.DESCENDING: self.act_descending,
            FSMState.TRAVERSING: self.act_traversing,
            FSMState.PENDING: self.act_pending, }

        self.map = np.zeros((FSMAction.MAX__, FSMState.MAX__), int)

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

    def state_str(self):
        str_dict = {
            FSMState.STOPPED: "STP",
            FSMState.ASCENDING: "ASC",
            FSMState.DESCENDING: "DES",
            FSMState.TRAVERSING: "TRV",
            FSMState.PENDING: "PND"}
        return str_dict[self.cur]

    def next(self, act: FSMAction):
        st = self.map[act][self.cur]
        self.cur = st

    def set(self, state: FSMState):
        self.cur = state

    def reset(self):
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
            start = p.cur.clone()
            start.t = 0
            end_time = TOP_TM_C * (p.target.t - start.t)

            end = DestPoint(
                [(start.x + END_OF_C * (p.target.x - start.x)),
                 (start.y + END_OF_C * (p.target.y - start.y)),
                 WALK_H],
                end_time, None, None, 0.0)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(DestPoint(
                [start.x + MID_OF_C * (p.target.x - start.x),
                 start.y + MID_OF_C * (p.target.y - start.y),
                 start.z + MID_PT_HT_C * (WALK_H - start.z)],
                end_time / 2))
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        if len(p.steps) > 0:
            # global misc_data
            # misc_data.append(p.steps[0].z)

            p.step()
        else:
            p.reset()
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
                [p.target.x, p.target.y, p.target.z], p.target.t, 0)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(DestPoint(
                [start.x + PRE_LAND_PT_OF_C * (p.target.x - start.x),
                 start.y + PRE_LAND_PT_OF_C * (p.target.y - start.y),
                 end.z + PRE_LAND_PT_HT_C * (WALK_H - end.z)],
                start.t + PRE_LAND_PT_TM_C * (p.target.t - start.t)))
            p.points.append(end)

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
            p.step()
        else:
            p.reset()
            self.next(FSMAction.END)

    def act_traversing(self):
        p = self.leg.plan

        if p.need_plan:
            print(f"{self.leg.name} : Traversing")
            self.leg.do_balance = True
            start = p.cur.clone()
            start.t = 0.0
            start.dx = 0.0
            start.dy = 0.0
            start.dz = 0.0
            end = DestPoint([p.target.x, p.target.y, p.target.z],
                            p.target.t, 0.0, 0.0, 0.0)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        if len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.END)
        else:
            balance = get_balance(self.leg)
            p.adjust(balance[0], balance[1], balance[2])
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
        p.adjust(balance[0], balance[1], balance[2])
        p.step_zero()
