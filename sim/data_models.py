
import pybullet as p
import numpy as np
from enum import IntEnum
import math
import functools
from numba import njit
from consts import cw_seq, LEG_TAR_H, MAX_DIP, T_RAD


@njit
def get_cross_product(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1],  a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


@njit
def get_mv_dot_product(mat, vec):
    return np.sum(mat * vec, axis=1)


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

    def get_json(self):
        return {
            "type": int(self.type),
            "pos": self.pos.round(5).tolist(),
        }


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
        self.base_orientation_matrix = np.zeros((3, 3), dtype=float)
        self.base_frame_orientation_matrix = np.zeros((3, 3), dtype=float)
        # self.base_orientation = np.zeros(4, dtype=float)
        # self.base_position = np.zeros(3, dtype=float)
        # self.horizontal_turn_matrix = np.zeros((3, 3), dtype=float)
        self.t_force_info: TForceInfo = TForceInfo(
            np.zeros(3, dtype=float), TouchState.PT0)
        self.s_center = np.zeros(3, dtype=float)
        # self.to_s_closest = np.zeros(3, dtype=float)

    def get_json(self):
        return {
            "avg_leg_h": round(self.avg_leg_h, 4),
            "touch_force": self.touch_force.round(4).tolist(),
            "damp": self.damp.round(4).tolist(),
            "bf_vec": self.base_force_vector.round(4).tolist(),
            "bfo_mat": self.base_frame_orientation_matrix.round(4).tolist(),
            "t_force": self.t_force_info.get_json(),
        }

    def get_base_frame_orientation_matrix(self):
        for i in range(4):
            if self.host.legs_cw[i] and self.host.legs_cw[i - 1] and self.host.legs_cw[i - 2]:
                forward = np.array([-1, 0, 0])
                X = get_mv_dot_product(self.base_orientation_matrix, forward)
                if X[0] <= 0:
                    temp = math.atan(X[1] / X[0])
                else:
                    temp = math.atan(X[1] / X[0]) - math.pi
                h = p.getMatrixFromQuaternion(
                    p.getQuaternionFromEuler([0, 0, -temp]))
                horizontal_turn_m = np.zeros((3, 3))
                for j in range(9):
                    horizontal_turn_m[j // 3][j % 3] = h[j]
                # self.horizontal_turn_matrix = horizontal_turn_m.copy()
                new_matrix = np.matmul(
                    horizontal_turn_m, self.base_orientation_matrix)
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
        # self.base_position = np.array(
        # p.getBasePositionAndOrientation(self.host.model)[0])

        # base and base frame orientation matrix
        # self.base_orientation = np.array(
        # p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.host.model)[1]))
        base_orientation_1d = np.array(p.getMatrixFromQuaternion(
            p.getBasePositionAndOrientation(self.host.model)[1]))
        for i in range(9):
            self.base_orientation_matrix[i // 3][i %
                                                 3] = base_orientation_1d[i]
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


class Leg:
    def __init__(self, name, base, shoulder, knee, heel, damp, sensor, dir, pos, off):
        ps = np.array(pos)
        bo = np.array(off)

        self.next: Leg = None
        self.prev: Leg = None
        self.idx: int = -1
        self.name: str = name
        self.body = None
        self.position = ps + bo
        self.def_pos = ps + bo
        self.base_off = bo
        self.joint_lims = None
        self.base = base
        self.shoulder = shoulder
        self.knee = knee
        self.heel = heel
        self.dampener = damp
        self.sensor = sensor
        self.grounded = True
        self.dir = dir
        self.sh_w = 0.02
        self.sh_h = 0.03
        self.link_len = 0.1
        self.damp_len = 0.012
        self.stiffness_c = 0.00005
        self.do_balance: bool = True

    def get_json(self):
        return {
            "name": self.name,
            "idx": self.idx,
            "pos": self.position.tolist(),
            "def_pos": self.def_pos.tolist(),
            "base_off": self.base_off.tolist(),
            "dir": self.dir,
            "joint_lims": self.joint_lims.round(4).tolist(),
        }


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

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self
            base_st = p.getJointInfo(self.model, l.base)
            shoulder_st = p.getJointInfo(self.model, l.shoulder)
            knee_st = p.getJointInfo(self.model, l.knee)
            l.joint_lims = np.array(
                list(base_st[8:10] + shoulder_st[8:10] + knee_st[8:10]))

        for i in range(-1, len(self.legs_cw) - 1):
            self.legs_cw[i].prev = self.legs_cw[i-1]
            self.legs_cw[i].next = self.legs_cw[i+1]

    def get_json(self):
        return {"legs": [l.get_json() for l in self.legs],
                "base": {
                    "max_dip": MAX_DIP,
                    "leg_tar_h": LEG_TAR_H,
                    "t_rad": T_RAD,
                    "cw": cw_seq,
                    "link_len": 0.1,
        }}
