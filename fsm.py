from enum import IntEnum

from plan import *
from plan import DestPoint


class FSMState(IntEnum):
    STOPPED = 0
    ASCENDING = 1
    DESCENDING = 2
    LANDING = 3
    TRAVERSING = 4
    DROPPING = 5
    PENDING = 6
    MAX__ = 7


class FSMAction(IntEnum):
    NO_ACT = 0
    END = 1
    HIT = 2
    DROP = 3
    MAX__ = 4


class FSM:
    def __init__(self, leg):
        self.leg = leg
        self.cur = FSMState.STOPPED
        self.func = {
            FSMState.STOPPED: self.act_stopped,
            FSMState.ASCENDING: self.act_ascending,
            FSMState.DESCENDING: self.act_descending,
            FSMState.LANDING: self.act_landing,
            FSMState.TRAVERSING: self.act_traversing,
            FSMState.DROPPING: self.act_dropping,
            FSMState.PENDING: self.act_pending,
        }

        self.map = np.zeros((FSMAction.MAX__, FSMState.MAX__), int)

        self.map[FSMAction.NO_ACT][FSMState.STOPPED] = FSMState.STOPPED
        self.map[FSMAction.END][FSMState.STOPPED] = FSMState.PENDING

        self.map[FSMAction.NO_ACT][FSMState.ASCENDING] = FSMState.ASCENDING
        self.map[FSMAction.END][FSMState.ASCENDING] = FSMState.DESCENDING

        self.map[FSMAction.NO_ACT][FSMState.DESCENDING] = FSMState.DESCENDING
        self.map[FSMAction.END][FSMState.DESCENDING] = FSMState.LANDING
        self.map[FSMAction.HIT][FSMState.DESCENDING] = FSMState.PENDING

        self.map[FSMAction.NO_ACT][FSMState.LANDING] = FSMState.LANDING
        self.map[FSMAction.HIT][FSMState.LANDING] = FSMState.PENDING

        self.map[FSMAction.NO_ACT][FSMState.TRAVERSING] = FSMState.TRAVERSING
        self.map[FSMAction.DROP][FSMState.TRAVERSING] = FSMState.DROPPING
        self.map[FSMAction.END][FSMState.TRAVERSING] = FSMState.PENDING

        self.map[FSMAction.NO_ACT][FSMState.DROPPING] = FSMState.DROPPING
        self.map[FSMAction.HIT][FSMState.DROPPING] = FSMState.TRAVERSING

        self.map[FSMAction.NO_ACT][FSMState.PENDING] = FSMState.PENDING

    def next(self, act: FSMAction):
        st = self.map[act][self.cur]
        self.cur = st

    def set(self, state: FSMState):
        self.cur = state

    def reset(self):
        self.cur = FSMState.STOPPED

    def execute(self):
        self.leg.plan.log.targets.append(self.leg.plan.target)
        while self.cur != FSMState.PENDING:
            self.func[self.cur]()

        print("Pending")

    def act_stopped(self):
        pass

    def act_ascending(self):
        TOP_TM_C = 0.4
        MID_PT_HT_C = 0.75
        END_OF_C = 0.5
        MID_OF_C = 0.25
        p = self.leg.plan

        if p.need_plan:
            print("Ascending")
            start = p.cur.clone()
            start.t = 0
            end_time = TOP_TM_C * (p.target.t - start.t)

            end = DestPoint(
                (start.x + END_OF_C * (p.target.x - start.x)),
                MAX_HEIGHT,
                end_time, None, 0.0)

            self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(DestPoint(
                (start.x + MID_OF_C * (p.target.x - start.x)),
                start.y + MID_PT_HT_C * (MAX_HEIGHT - start.y),
                end_time / 2))
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        if len(p.steps) > 0:
            p.step()
        else:
            p.reset()
            self.next(FSMAction.END)

    def act_descending(self):
        PRE_LAND_PT_HT_C = 0.2
        PRE_LAND_PT_OF_C = 0.96
        PRE_LAND_PT_TM_C = 0.7
        p = self.leg.plan

        if p.need_plan:
            print("Descending")
            start = p.cur.clone()
            end = DestPoint(p.target.x, p.target.y, p.target.t, 0)

            self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(DestPoint(
                start.x + PRE_LAND_PT_OF_C * (p.target.x - start.x),
                end.y + PRE_LAND_PT_HT_C * (MAX_HEIGHT - end.y),
                start.t + PRE_LAND_PT_TM_C * (p.target.t - start.t)))
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        hits, _ = p.check2()

        if hits:
            p.reset()
            self.next(FSMAction.HIT)
        elif len(p.steps) > 0:
            p.step()
        else:
            p.reset()
            self.next(FSMAction.END)

    def act_landing(self):
        p = self.leg.plan

        if p.need_plan:
            print("Landing")
            start = p.cur.clone()
            start.t = 0
            dy = abs(p.last.y - p.cur.y)
            dt = abs(p.last.t - p.cur.t)
            end = DestPoint(p.target.x, MAX_DIP, abs(start.y - MAX_DIP) * dy / dt)

            self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        hits, _ = p.check2()

        if hits or len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.HIT)
        else:
            p.step()

    def act_traversing(self):
        p = self.leg.plan

        if p.need_plan:
            print("Traversing")
            start = p.cur.clone()
            start.t = 0
            start.dx = 0
            start.dy = 0
            end = DestPoint(p.target.x, p.target.y, p.target.t, 0.0, 0.0)

            self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        hits, adj = p.check2()

        if len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.END)
        elif not hits:
            p.reset()
            self.next(FSMAction.DROP)
        elif ADJUST_SENS < abs(adj):
            p.adjust(0, adj)
            p.step()
        else:
            p.step()

    def act_dropping(self):
        p = self.leg.plan

        if p.need_plan:
            print("Dropping")
            start = p.cur.clone()
            start.t = 0.0
            # dy = abs(p.last.y - p.cur.y)
            # dt = abs(p.last.t - p.cur.t)
            end = DestPoint(p.cur.x, MAX_DIP, drop_time(start.y))

            self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        hits, _ = p.check2()

        if hits or len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.HIT)
        else:
            p.step()

    def act_staying(self):
        pass

    def act_pending(self):
        print("Pending")
