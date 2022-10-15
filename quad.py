from leg import Leg
from consts import *
import pybullet as p
import numpy as np
import math
import functools
from enum import IntEnum
from interp import gradual_speed_func, variable_speed_func, do_nothing, get_cross_product, map_ranges, get_mv_dot_product
from plan import DestPoint
# from ground_view import SPoint
from fsm import FSMState, FSMAction, FSM


class TouchState(IntEnum):
    PT3 = 0
    PT2 = 1
    PT1 = 2
    PT0 = 3


t_state_color_tbl = {
    TouchState.PT3: "green",
    TouchState.PT2: "yellow",
    TouchState.PT1: "orange",
    TouchState.PT0: "red",
}


class TForceInfo:
    def __init__(self, pos: np.ndarray, state: TouchState):
        self.pos = pos
        self.type: TouchState = state
        self.color: str = t_state_color_tbl[state]

    def clone(self):
        return TForceInfo(np.copy(self.pos), self.type)

# class Quad_Snd:
    # def __init__(q: Quad)


class SensInfoView:
    def __init__(self, si):
        self.avg_leg_h: float = si.avg_leg_h
        self.abs_std_leg_h: float = si.abs_std_leg_h
        self.touch_force = si.touch_force.copy()
        self.damp = si.damp.copy()
        self.base_force_vector = si.base_force_vector.copy()
        self.bf_hist = [arr.copy() for arr in si.bf_hist]
        self.bf_max = si.bf_max
        self.base_orientation_matrix = si.base_orientation_matrix.copy()
        self.base_frame_orientation_matrix = si.base_frame_orientation_matrix.copy()
        self.base_orientation = [arr.copy() for arr in si.base_orientation]
        self.base_position = [arr.copy() for arr in si.base_position]
        self.horizontal_turn_matrix = si.horizontal_turn_matrix.copy()
        self.t_force_info: TForceInfo = si.t_force_info.clone()
        self.s_center = si.s_center.copy()
        self.to_s_closest = si.to_s_closest.copy()


class SensInfo:
    def __init__(self, q):
        self.host: Quad = q
        self.avg_leg_h: float = LEG_TAR_H
        self.abs_std_leg_h: float = 0.0
        self.touch_force = np.zeros(4, dtype=int)
        self.damp = np.zeros(4, dtype=float)
        self.base_force_vector = np.zeros(3, dtype=float)
        self.bf_hist = [np.zeros(3, dtype=float)]
        self.bf_max = 35
        self.base_orientation_matrix = [np.zeros((3, 3), dtype=float)]
        self.base_frame_orientation_matrix = [np.zeros((3, 3), dtype=float)]
        self.base_orientation = [np.zeros(4, dtype=float)]
        self.base_position = [np.zeros(3, dtype=float)]
        self.horizontal_turn_matrix = [np.zeros((3, 3), dtype=float)]
        self.t_force_info: TForceInfo = TForceInfo(
            np.zeros(3, dtype=float), TouchState.PT0)
        self.s_center = np.zeros(3, dtype=float)
        self.to_s_closest = np.zeros(3, dtype=float)

    def get_view(self):
        return SensInfoView(self)

    # # some math, nothing to see here

    def get_base_frame_orientation_matrix(self):
        for i in range(4):
            if self.host.legs_cw[i] and self.host.legs_cw[i - 1] and self.host.legs_cw[i - 2]:
                forward = np.array([-1, 0, 0])
                X = get_mv_dot_product(base_orientation_matrix, forward)
                # X = base_orientation_matrix.dot(forward)
                if X[0] <= 0:
                    temp = math.atan(X[1] / X[0])
                else:
                    temp = math.atan(X[1] / X[0]) - math.pi
                h = p.getMatrixFromQuaternion(
                    p.getQuaternionFromEuler([0, 0, -temp]))
                horizontal_turn_m = np.zeros((3, 3))
                for j in range(9):
                    horizontal_turn_m[j // 3][j % 3] = h[j]
                self.horizontal_turn_matrix = horizontal_turn_m.copy()
                new_matrix = np.matmul(
                    horizontal_turn_m, base_orientation_matrix)
                return new_matrix
        return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def update_s(self):
        t_pos = []
        l: Leg
        for i in range(-1, len(self.host.legs)-1):
            l = self.host.legs[cw_seq[i]]
            if self.touch_force[l.idx] > 0:
                t_pos.append(l.position)

        if len(t_pos) == 0:
            return

        center = functools.reduce(lambda a, b: a + b, t_pos) / len(t_pos)
        self.s_center = center

    def update_t_force(self):
        force = np.copy(self.host.sens_info.base_force_vector)
        if np.linalg.norm(force) != 0:
            force = force / np.linalg.norm(force)

        res_arr = []
        state_arr = []
        legs = self.host.legs_cw
        state = TouchState.PT0
        for i in range(4):
            la: Leg = legs[i]
            lb: Leg = legs[i - 1]
            lc: Leg = legs[i - 2]
            ba = lb.position - la.position
            bc = lb.position - lc.position
            norm = get_cross_product(bc, ba)
            # norm = strech_vector_to(norm, 1.0)
            d = -1 * np.sum(lb.position * norm)
            f_sum = np.sum(norm * force)
            mult = -d / (f_sum if f_sum else 1)
            res = mult * force
            res_arr.append(res)

            f0 = self.touch_force[cw_seq[i]]
            f1 = self.touch_force[cw_seq[i - 1]]
            f2 = self.touch_force[cw_seq[i - 2]]

            if f0 > 0.0 and f1 > 0.0 and f2 > 0.0:
                state_arr.append(TouchState.PT3)
            elif f1 > 0.0 and (f0 > 0.0 or f2 > 0.0):
                state_arr.append(TouchState.PT2)
            elif f0 > 0.0 or f1 > 0.0 or f2 > 0.0:
                state_arr.append(TouchState.PT1)
            else:
                state_arr.append(TouchState.PT0)

        if TouchState.PT3 in state_arr:
            reduced_arr = [j for i, j in zip(
                state_arr, res_arr) if i == TouchState.PT3]
            state = TouchState.PT3
        elif TouchState.PT2 in state_arr:
            reduced_arr = [j for i, j in zip(
                state_arr, res_arr) if i == TouchState.PT2]
            state = TouchState.PT2
        elif TouchState.PT1 in state_arr:
            reduced_arr = [j for i, j in zip(
                state_arr, res_arr) if i == TouchState.PT1]
            state = TouchState.PT1
        else:
            reduced_arr = res_arr

        if len(res_arr) > 0:
            reduced = functools.reduce(
                lambda a, b: a + b, reduced_arr) / len(reduced_arr)
            self.t_force_info = TForceInfo(reduced, state)

    def update(self):
        # base position
        self.base_position = np.array(
            p.getBasePositionAndOrientation(self.host.model)[0])

        # base and base frame orientation matrix
        self.base_orientation = np.array(
            p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.host.model)[1]))
        base_orientation_1d = np.array(p.getMatrixFromQuaternion(
            p.getBasePositionAndOrientation(self.host.model)[1]))
        for i in range(9):
            base_orientation_matrix[i // 3][i % 3] = base_orientation_1d[i]
        self.base_frame_orientation_matrix = self.get_base_frame_orientation_matrix()

        # damp and touch force
        for i, l in enumerate(self.host.legs):
            self.damp[l.idx] = p.getJointState(
                self.host.model, l.dampener)[0] / T_RAD
            f = p.getJointState(self.host.model, l.sensor)[2][2]
            if np.isnan(f):
                f = 0.0
                print(f"force was nan")
                # TODO: I somehow receive this, when clearing leg position <07-10-22, yourname> #
            self.touch_force[l.idx] = -f

        # base force vector
        b_force = -np.array(p.getJointState(self.host.model,
                            self.host.sensor)[2][slice(3)])
        if len(self.bf_hist) >= self.bf_max:
            self.bf_hist.pop(0)

        # IDK whqt this is
        self.bf_hist.append(b_force)

        # 'base_force_vector' is running average of last 'bf_max' b_force' values
        self.base_force_vector = functools.reduce(lambda a, b: a + b, self.bf_hist) / len(
            self.bf_hist)

        # touch force vector
        self.update_t_force()

        self.update_s()

        # average balanced leg height
        ahl = [l.position[2] for l in self.host.legs if l.do_balance]
        self.avg_leg_h = np.average(np.array(ahl)) if len(ahl) else MAX_DIP

        # absolute std of balanced leg heights
        self.abs_std_leg_h = np.average(np.array(
            [abs(l.position[2] - self.avg_leg_h) for l in self.host.legs if l.do_balance])) if len(ahl) else 0.0


class QuadView:
    def __init__(self, q):
        self.sens_info = q.sens_info.get_view()
        self.front_left_leg: Leg = q.front_left_leg.get_view()
        self.front_right_leg: Leg = q.front_right_leg.get_view()
        self.back_left_leg: Leg = q.back_left_leg.get_view()
        self.back_right_leg: Leg = q.back_right_leg.get_view()

        self.legs = [self.front_left_leg, self.front_right_leg,
                     self.back_left_leg, self.back_right_leg]

        self.legs_cw = [self.front_left_leg, self.front_right_leg,
                        self.back_right_leg, self.back_left_leg]

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self

        for i in range(-1, len(self.legs_cw) - 1):
            self.legs_cw[i].prev = self.legs_cw[i-1]
            self.legs_cw[i].next = self.legs_cw[i+1]


class Quad:
    def __init__(self, model, fll, frl, bll, brl, sensor):
        self.model = model
        self.sens_info = SensInfo(self)
        self.sensor = sensor
        self.front_left_leg: Leg = fll
        self.front_right_leg: Leg = frl
        self.back_left_leg: Leg = bll
        self.back_right_leg: Leg = brl

        self.legs = [fll, frl, bll, brl]
        self.legs_cw = [fll, frl, brl, bll]
        self.sensors = [fll.sensor, frl.sensor, bll.sensor, brl.sensor, sensor]

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self

        for i in range(-1, len(self.legs_cw) - 1):
            self.legs_cw[i].prev = self.legs_cw[i-1]
            self.legs_cw[i].next = self.legs_cw[i+1]

    def get_view(self):
        return QuadView(self)

    def get_path_len(self, task) -> float:
        pos_lst = [self.legs[task.idx].plan.targt.pos]
        pos_lst.extend([p.pos for p in task.points])
        pos_difs = [np.linalg.norm(p1 - p0)
                    for p0, p1 in zip(pos_lst[:-1], pos_lst[1:])]
        S = np.sum(np.array(pos_difs))
        return S

    def get_task_act(self, task):
        l = self.legs[task.idx]
        if len(task.points) == 0:
            return FSMState.PENDING

        if task.do_lift:
            act = FSMState.ASCENDING
            t_pt = task.points[-1]
        else:
            act = FSMState.TRAVERSING
            t_pt = task.points[0]

        S = np.linalg.norm(t_pt.pos - l.plan.target.pos)

        if S == 0:
            act = FSMState.PENDING

        return act

    def prepare_pending_points(self, task):
        l = self.legs[task.idx]
        pts = [DestPoint(np.copy(l.plan.target.pos), f_arr_unset(),
                         ts=0, vel_ps=0.0)]
        return pts

    def prepare_ascending_points(self, task, n_steps):
        pts = [DestPoint(task.points[-1].pos, f_arr_unset(),
                         ts=n_steps)]
        return pts

    def prepare_traversing_points(self, task, speed_ps: float):
        l = self.legs[task.idx]

        # Getting positions of every path point
        pos_lst = [self.legs[task.idx].plan.target.pos]
        pos_lst.extend([p.pos for p in task.points])

        # Getting lengths of every path interval
        pos_difs = [0.0] + [np.linalg.norm(p1 - p0)
                            for p0, p1 in zip(pos_lst[:-1], pos_lst[1:])]
        # S = np.sum(np.array(pos_difs))

        # Aggregating lengths of every path point
        pos_dif_agr = []
        pos_dif_agr_last = 0.0
        for pd in pos_difs:
            pos_dif_agr.append(pos_dif_agr_last + pd)
            pos_dif_agr_last = pos_dif_agr[-1]

        # Getting velocities of every path point
        vels = map_ranges(
            pos_dif_agr, pos_dif_agr[0], pos_dif_agr[-1], l.plan.target.vel_ps, speed_ps, default=speed_ps)
        # Getting number of steps of every path point
        n_steps_unrounded = [
            2 * s / (v0 + v1) for s, v0, v1 in zip(pos_difs[1:], vels[:-1], vels[1:])]

        # n_steps_unrounded = [2 * s / (v0 + v1) for s, v0, v1 in zip(pos_difs, np.array([l.plan.target.vel_ps] + list(vels[:-1])), vels) ]
        # Aggregating number of steps of every path point
        n_steps_unr_agr = []
        n_steps_unr_agr_last = 0.0
        for s in n_steps_unrounded:
            n_steps_unr_agr.append(n_steps_unr_agr_last + s)
            n_steps_unr_agr_last = n_steps_unr_agr[-1]

        # Rounding aggregated number of steps of every path point
        n_steps_rounded_agr = [round(s) for s in n_steps_unr_agr]

        # Getting DestPoint of every path point
        pts = [DestPoint(p, f_arr_unset(), ts=s, vel_ps=vps)
               for p, s, vps in zip(pos_lst[1:], n_steps_rounded_agr, vels[1:])]

        return pts

    def set_target(self, tasks):
        # init_pace = 5.0
        # init_pace = 1.6
        # init_pace = 0.7
        speed_ps = 0.001

        for t in tasks:
            l = self.legs[t.idx]
            S = 0.0
            est_n_steps = 0

            act = self.get_task_act(t)

            match act:
                case FSMState.PENDING:
                    speed_func = do_nothing
                    pts = self.prepare_pending_points(t)

                case FSMState.ASCENDING:
                    # est_n_steps = 500
                    # est_n_steps = 10
                    est_n_steps = 100
                    speed_func = variable_speed_func
                    pts = self.prepare_ascending_points(t, est_n_steps)

                case FSMState.TRAVERSING:
                    speed_func = gradual_speed_func
                    pts = self.prepare_traversing_points(t, speed_ps)

            l.make_plan(pts, act, speed_func)
