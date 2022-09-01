from enum import IntEnum
import functools

from plan import *
from balance import *
from plan import DestPoint

min_damp = 0.0
max_damp = 0.0


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
            FSMState.PENDING: self.act_pending, }

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
        self.map[FSMAction.END][FSMState.DROPPING] = FSMState.PENDING

        self.map[FSMAction.NO_ACT][FSMState.PENDING] = FSMState.PENDING
        self.map[FSMAction.DROP][FSMState.PENDING] = FSMState.DROPPING

    def state_str(self):
        str_dict = {
            FSMState.STOPPED: "STP",
            FSMState.ASCENDING: "ASC",
            FSMState.DESCENDING: "DES",
            FSMState.LANDING: "LND",
            FSMState.TRAVERSING: "TRV",
            FSMState.DROPPING: "DRP",
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
                [(start.x + END_OF_C * (p.target.x - start.x)),
                 (start.y + END_OF_C * (p.target.y - start.y)),
                 MAX_HEIGHT],
                end_time, None, None, 0.0)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(DestPoint(
                [start.x + MID_OF_C * (p.target.x - start.x),
                 start.y + MID_OF_C * (p.target.y - start.y),
                 start.z + MID_PT_HT_C * (MAX_HEIGHT - start.z)],
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
            end = DestPoint(
                [p.target.x, p.target.y, p.target.z], p.target.t, 0)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(DestPoint(
                [start.x + PRE_LAND_PT_OF_C * (p.target.x - start.x),
                 start.y + PRE_LAND_PT_OF_C * (p.target.y - start.y),
                 end.z + PRE_LAND_PT_HT_C * (MAX_HEIGHT - end.z)],
                start.t + PRE_LAND_PT_TM_C * (p.target.t - start.t)))
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        _, hits, _ = self.leg.check_damp()

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
            dz = abs(p.last.z - p.cur.z)
            dt = abs(p.last.t - p.cur.t)
            end = DestPoint([p.target.x, p.target.y, MAX_DIP],
                            abs(start.z - MAX_DIP) * dz / dt)

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        _, hits, _ = self.leg.check_damp()

        if hits or len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.HIT)
        else:
            p.step()

    def act_traversing(self):
        p = self.leg.plan
        p.adjust(0, 0, 0.05 * (LEG_TAR_H - self.leg.position[2]))

        if p.need_plan:
            print(f"{p.leg.name}:Traversing")
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

        _, hits, adj = self.leg.check_damp()

        if len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.END)
        elif not hits:
            p.reset()
            self.next(FSMAction.DROP)
        elif ADJUST_SENS < abs(adj):
            p.adjust(0, 0, adj)
            # print(f"adjusted by {[0, 0, adj]}")
            p.step()
        else:
            p.step()

    def act_dropping(self):
        p = self.leg.plan
        p.adjust(0, 0, 0.05 * (LEG_TAR_H - self.leg.position[2]))

        if p.need_plan:
            print(f"{p.leg.name}:Dropping")
            start = p.cur.clone()
            start.t = 0.0
            # print(f"led:{p.leg.name} h:{start.z} drop time:{drop_time(start.z)}")
            end = DestPoint([p.cur.x, p.cur.y, MAX_DIP], drop_time(start.z))

            # self.leg.plan.log.points.append(end)

            p.points.append(start)
            p.points.append(end)

            fill_diffs(p.points)

            p.plan_steps()

            p.need_plan = False

        _, hits, _ = self.leg.check_damp()

        if hits:
            p.reset()
            self.next(FSMAction.HIT)
        elif len(p.steps) == 0:
            p.reset()
            self.next(FSMAction.END)
        else:
            p.step()

    def act_stopped(self):
        pass

    def act_pending(self):
        p = self.leg.plan

        balance = get_balance(self.leg)
        p.set_adjust(0, 0, balance)
        p.step_zero()
        

        # adj_list = []

        # p = self.leg.plan
        # hits, adj = self.leg.check_damp()

        # x_pos = [l.position[0] for l in self.leg.body.legs]
        # x_pos.append(self.leg.body.sens_info.t_force_info.pos[0])
        # min_x_pos = functools.reduce(lambda a, b: a if a < b else b, x_pos)
        # max_x_pos = functools.reduce(lambda a, b: a if a > b else b, x_pos)
        # x_pos_diff = abs(max_x_pos - min_x_pos)
        # x_lever_inv = abs(
            # self.leg.position[0] - self.leg.body.sens_info.t_force_info.pos[0]) / x_pos_diff
        # x_lever = 1 / x_lever_inv

        # y_pos = [l.position[1] for l in self.leg.body.legs]
        # y_pos.append(self.leg.body.sens_info.t_force_info.pos[1])
        # min_y_pos = functools.reduce(lambda a, b: a if a < b else b, y_pos)
        # max_y_pos = functools.reduce(lambda a, b: a if a > b else b, y_pos)
        # y_pos_diff = abs(max_y_pos - min_y_pos)
        # y_lever_inv = abs(
            # self.leg.position[1] - self.leg.body.sens_info.t_force_info.pos[1]) / y_pos_diff
        # y_lever = 1 / y_lever_inv

        # leg_imp_share = x_lever * y_lever
        # leg_inv_imp_share = x_lever_inv * y_lever_inv

        # p.adjust(0, 0, leg_inv_imp_share *
                 # 0.06 * (LEG_TAR_H - self.leg.body.avg_leg_h))
        # adj_list.append(leg_inv_imp_share * 0.06 *
                        # (LEG_TAR_H - self.leg.body.avg_leg_h))

        # force_r_mat = get_rotation_matrix_from_two_vectors(
            # np.array([0.0, 0.0, -1.0]), self.leg.body.sens_info.base_force_vector)
        # leg_pos_mod = np.dot(self.leg.position, force_r_mat)
        # leg_dif = leg_pos_mod - self.leg.position

        # force_to_leg_angle = get_vectors_angle(
            # self.leg.body.sens_info.base_force_vector, self.leg.position)
        # horizontal_leg_len = np.linalg.norm(self.leg.position[:2])
        # leg_hight_off = math.tan(force_to_leg_angle) * horizontal_leg_len

        # # p.adjust(0, 0, 0.012 * leg_dif[2])
        # # adj_list.append(0.012 * leg_dif[2])
        # if hits:
            # p.adjust(0, 0, 0.012 * leg_dif[2])
            # adj_list.append(0.012 * leg_dif[2])

        # else:
            # p.adjust(0, 0, 0.06 * (self.leg.body.avg_leg_h - self.leg.position[2])
                     # )
            # adj_list.append(
                # 0.06 * (self.leg.body.avg_leg_h - self.leg.position[2]))

        # # if self.leg.name in ["front_left", "front_right"]:
            # # d = adj_sign * adj_len
            # # print(f"avg:{self.leg.body.avg_leg_h} adj:{self.leg.plan.adj[2]} value:{d}")

        # if hits and ADJUST_SENS < abs(adj):
            # p.adjust(0, 0, 0.4 * adj)
            # adj_list.append(0.4 * adj)
            # p.step_zero()
        # elif not hits:
            # p.adjust(0, 0, -0.4 * T_RAD)
            # adj_list.append(-0.4 * T_RAD)
            # p.step_zero()
        # else:
            # p.step_zero()

        # if self.leg.name in ["front_left", "front_right"]:
            # print(f"{self.leg.name} imp:{leg_imp_share} ah:{self.leg.body.avg_leg_h} avg:{np.average(np.array(adj_list))} list:{adj_list}")

        # # print("Pending")
