from leg import Leg
from consts import *
import pybullet as p
import numpy as np
import math
import functools
from enum import IntEnum
from interp import *
# from ground_view import SPoint
from fsm import *


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


class SensInfo:
    def __init__(self, q):
        self.host: Quad = q
        self.touch_force = np.zeros(4, dtype=int)
        self.base_force_vector = np.zeros(3, dtype=float)
        self.bf_hist = [np.zeros(3, dtype=float)]
        self.bf_max = 35
        self.base_orientation_matrix = None
        self.base_frame_orientation_matrix = None
        self.base_orientation = [np.zeros(4, dtype=float)]
        self.base_position = [np.zeros(3, dtype=float)]
        self.horizontal_turn_matrix = None
        self.t_force_info: TForceInfo = TForceInfo(
            np.zeros(3, dtype=float), TouchState.PT0)
        self.s_center = np.zeros(3, dtype=float)
        self.to_s_closest = np.zeros(3, dtype=float)

    # # some math, nothing to see here
    def get_base_frame_orientation_matrix(self):
        for i in range(4):
            if self.host.legs_cw[i] and self.host.legs_cw[i - 1] and self.host.legs_cw[i - 2]:
                forward = np.array([-1, 0, 0])
                X = base_orientation_matrix.dot(forward)
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
        self.avg_leg_h: float = LEG_TAR_H

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self

        for i in range(-1, len(self.legs_cw) - 1):
            self.legs_cw[i].prev = self.legs_cw[i-1]
            self.legs_cw[i].next = self.legs_cw[i+1]

    # gets info from sensors

    def update_sensor_info(self):
        self.sens_info.base_position = np.array(
            p.getBasePositionAndOrientation(self.model)[0])
        self.sens_info.base_orientation = np.array(
            p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.model)[1]))
        base_orientation_1d = np.array(p.getMatrixFromQuaternion(
            p.getBasePositionAndOrientation(self.model)[1]))
        for i in range(9):
            base_orientation_matrix[i // 3][i % 3] = base_orientation_1d[i]
        self.sens_info.base_frame_orientation_matrix = self.sens_info.get_base_frame_orientation_matrix()
        for i, l in enumerate(self.legs):
            f = p.getJointState(self.model, l.sensor)[2][2]
            if np.isnan(f):
                f = 0.0
                print(f"force was nan")
            self.sens_info.touch_force[l.idx] = -f
            # self.sens_info.touch_force[l.idx] = -p.getJointState(self.model, l.sensor)[2][2]

        b_force = -np.array(p.getJointState(self.model,
                            self.sensor)[2][slice(3)])
        if len(self.sens_info.bf_hist) >= self.sens_info.bf_max:
            self.sens_info.bf_hist.pop(0)

        self.sens_info.bf_hist.append(b_force)
        # 'base_force_vector' is running average of last 'bf_max' b_force' values
        self.sens_info.base_force_vector = functools.reduce(lambda a, b: a + b, self.sens_info.bf_hist) / len(
            self.sens_info.bf_hist)
        self.sens_info.update_t_force()
        self.sens_info.update_s()

    def set_target(self, lst):
        if len(lst) == 0:
            target = np.zeros(3, dtype=float)
        else:
            target = lst[0]  # for now

        for l in self.legs:
            l_target = l.def_pos + target
            l.make_plan(DestPoint(l_target, 1.0), FSMState.TRAVERSING)
