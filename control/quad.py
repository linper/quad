from leg import Leg
from consts import *
import pybullet as p
import numpy as np
import math
import functools
from stance import Stance
from enum import IntEnum
from interp import gradual_speed_func, variable_speed_func, do_nothing, get_cross_product, map_ranges, get_mv_dot_product, strech_vector_to, get_3_from_2_arr
from plan import DestPoint, prepare_pending_points, prepare_ascending_points, prepare_traversing_points
# from ground_view import SPoint
from fsm import FSMState, FSMAction, FSM
import matplotlib.pyplot as plt

pred_acc = []
real_acc = []


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
        self.b_sensor_mass: float = p.getDynamicsInfo(q.model, q.sensor)[0]
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
                            self.host.sensor)[2][slice(3)]) / self.b_sensor_mass
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

        global real_acc, pred_acc

        if False and len(pred_acc) > 0:
            if len(pred_acc) == len(real_acc):
                plt.figure()
                time_data = list(range(len(pred_acc)))

                plt.plot(time_data, [a[0] for a in pred_acc], color="red")
                plt.plot(time_data, [a[1] for a in pred_acc], color="green")
                # plt.plot(time_data, [a[2] for a in pred_acc], color="blue")

                plt.plot(time_data, [a[0] for a in real_acc], color="yellow")
                plt.plot(time_data, [a[1] for a in real_acc], color="purple")
                # plt.plot(time_data, [a[2] for a in real_acc], color="brown")
                plt.grid("both")
                plt.show()
            else:
                real_acc.append(self.base_force_vector)


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

        self.dummy_leg = q.dummy_leg.get_view()
        self.dummy_leg.body = self

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self

        for i in range(-1, len(self.legs_cw) - 1):
            self.legs_cw[i].prev = self.legs_cw[i-1]
            self.legs_cw[i].next = self.legs_cw[i+1]


class Quad:
    def __init__(self, model, fll, frl, bll, brl, sensor):
        self.model = model
        self.sensor = sensor
        self.sens_info = SensInfo(self)
        self.front_left_leg: Leg = fll
        self.front_right_leg: Leg = frl
        self.back_left_leg: Leg = bll
        self.back_right_leg: Leg = brl

        self.legs = [fll, frl, bll, brl]
        self.legs_cw = [fll, frl, brl, bll]
        self.sensors = [fll.sensor, frl.sensor, bll.sensor, brl.sensor, sensor]
        self.dummy_leg = Leg.dummy()
        self.dummy_leg.body = self

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self

        for i in range(-1, len(self.legs_cw) - 1):
            self.legs_cw[i].prev = self.legs_cw[i-1]
            self.legs_cw[i].next = self.legs_cw[i+1]

    def get_view(self):
        return QuadView(self)

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

    def set_target(self, cmd):
        tasks = cmd.tasks
        speed_ps = 0.001
        n_opt_stances = 4

        com_task = cmd.tasks[0]

        if cmd.test_mode:
            com_target = com_task.points[0].pos[:2] if len(
                com_task.points) > 0 else np.array([0.0, 0.0])
            l_pos = np.array([l.position[:2] for l in self.legs])
            l_def_pos = np.array([l.def_pos[:2] for l in self.legs])
            st = Stance(l_pos, l_def_pos, com_task.direction,
                        com_target)
            # act, steps = st.plot()
            act, steps = st.get_movement(n_opt_stances)

            for l in self.legs:
                if act == FSMState.PENDING:
                    print(f"STANCE TEST: PENDING")
                    pts = prepare_pending_points(l.plan.target)
                    l.make_plan(pts, act, do_nothing)

                elif act == FSMState.ASCENDING and l.idx == st.next.pidx:
                    print(f"STANCE TEST: ASCENDING")
                    print(f"LEG POS:{l.position}")
                    print(f"P0:{st.pts[st.next.pidx]}")
                    print(f"P1:{st.next.pts[st.next.pidx]}")
                    ap = st.next.pts[st.next.pidx]
                    pts = prepare_ascending_points(
                        np.array([ap[0], ap[1], l.position[2]]), len(steps))
                    l.make_plan(pts, FSMState.ASCENDING, variable_speed_func)

                elif act in [FSMState.TRAVERSING, FSMState.ASCENDING]:
                    print(f"STANCE TEST: TRAVERSING")
                    print(f"LEG POS:{l.position}")
                    print(f"P0:{steps[0].pos}")
                    print(f"P1:{steps[-1].pos}")
                    steps_cl = [s.clone() for s in steps]
                    l.make_plan2(steps_cl, FSMState.TRAVERSING)

            return

        for t in tasks:
            if t.idx == -1:
                continue

            l = self.legs[t.idx]

            # if com_task.do_lift and len(com_task.points) > 0:
            if not cmd.test_mode and len(com_task.points) > 0:
                if not t.exp_set:
                    t.points.clear()
                    for p in com_task.points:
                        pc = p.clone()
                        pc.pos = - pc.pos + l.position
                        # pc.pos = - pc.pos + l.def_pos
                        t.points.append(pc)

            # S = 0.0
            est_n_steps = 0

            act = self.get_task_act(t)

            match act:
                case FSMState.PENDING:
                    speed_func = do_nothing
                    # pts = self.prepare_pending_points(t)
                    pts = prepare_pending_points(self.legs[t.idx].plan.target)

                case FSMState.ASCENDING:
                    # est_n_steps = 500
                    # est_n_steps = 10
                    est_n_steps = 100
                    speed_func = variable_speed_func
                    pts = prepare_ascending_points(
                        t.points[-1].pos, est_n_steps)

                case FSMState.TRAVERSING:
                    speed_func = gradual_speed_func
                    l = self.dummy_leg if t.idx == -1 else self.legs[t.idx]
                    pts = prepare_traversing_points(
                        l.plan.target, t.points, speed_ps)
                    pts = pts[1:]
                    # pts = self.prepare_traversing_points(t, speed_ps)

            l.make_plan(pts, act, speed_func)