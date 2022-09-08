
from enum import IntEnum
from pidc import PIDC2
import functools

import numpy as np

from plan import *


class BalanceAttrs(IntEnum):
    HEIGHT = 0
    LEAN = 1
    IMP = 2
    IMP_INV = 3
    TOUCH = 4
    DROP = 5
    MAX = 6


def get_importance(leg):
    x_pos = [l.position[0] for l in leg.body.legs]
    x_pos.append(leg.body.sens_info.t_force_info.pos[0])
    min_x_pos = functools.reduce(lambda a, b: a if a < b else b, x_pos)
    max_x_pos = functools.reduce(lambda a, b: a if a > b else b, x_pos)
    x_pos_diff = abs(max_x_pos - min_x_pos)
    x_lever_inv = abs(
        leg.position[0] - leg.body.sens_info.t_force_info.pos[0]) / x_pos_diff
    x_lever = 1 / x_lever_inv

    y_pos = [l.position[1] for l in leg.body.legs]
    y_pos.append(leg.body.sens_info.t_force_info.pos[1])
    min_y_pos = functools.reduce(lambda a, b: a if a < b else b, y_pos)
    max_y_pos = functools.reduce(lambda a, b: a if a > b else b, y_pos)
    y_pos_diff = abs(max_y_pos - min_y_pos)
    y_lever_inv = abs(
        leg.position[1] - leg.body.sens_info.t_force_info.pos[1]) / y_pos_diff
    y_lever = 1 / y_lever_inv

    leg_imp_share = x_lever * y_lever
    leg_inv_imp_share = x_lever_inv * y_lever_inv

    return leg_imp_share, leg_inv_imp_share


def get_lean_diff(leg):
    if leg.body.sens_info.damp[leg.idx] < SOFT_HIT_THR:
        return leg.body.avg_leg_h - leg.position[2]

    force_r_mat = get_rotation_matrix_from_two_vectors(
        np.array([0.0, 0.0, -1.0]), leg.body.sens_info.base_force_vector)
    leg_pos_mod = np.dot(leg.position, force_r_mat)
    leg_dif = leg_pos_mod - leg.position

    return leg_dif[2]


def get_balance_base(q, cof):
    res = np.zeros(4, dtype=float)

    bfo_mat = get_4x4_from_3x3_mat(q.sens_info.base_frame_orientation_matrix
                                   )
    height_mat = np.identity(4, dtype=float)

    height_mat[2, 3] = LEG_TAR_H - q.avg_leg_h  # making shift matrix in z axis

    for i, l in enumerate(q.legs):
        if q.sens_info.damp[l.idx] < SOFT_HIT_THR:
            res[i] = cof * (l.body.avg_leg_h - l.position[2])
            continue

        leg_pos_mod = np.ones(4, dtype=float)
        leg_pos_mod[:3] = l.position
        leg_pos_mod = bfo_mat.dot(leg_pos_mod)
        leg_pos_mod = height_mat.dot(leg_pos_mod)

        leg_dif = leg_pos_mod[:3] - l.position

        res[i] = (leg_dif * cof)[2]

    return res


def get_touch_diff(leg):
    h_hits, s_hits, adj = leg.check_damp()
    drop = 0.0
    touch = 0.0

    if s_hits and ADJUST_SENS < abs(adj):
        touch = adj
    elif not s_hits:
        drop = -T_RAD

    return drop, touch


balance_cache = np.zeros(4, dtype=float)
balance_manager = np.zeros(4)


def get_balance(leg):
    global balance_cache, balance_manager

    if balance_manager[leg.idx] == 1:
        balance_manager = np.zeros(4)
        balance_cache = np.zeros(4, dtype=float)

        balance_diffs = np.zeros((BalanceAttrs.MAX, 4), dtype=float)

        base_part = get_balance_base(leg.body, 1.0)

        for i, l in enumerate(leg.body.legs):
            balance_diffs[BalanceAttrs.DROP,
                          i], balance_diffs[BalanceAttrs.TOUCH, i] = get_touch_diff(l)
            balance_diffs[BalanceAttrs.LEAN, i] = get_lean_diff(l)
            _, balance_diffs[BalanceAttrs.IMP_INV, i] = get_importance(l)
            balance_diffs[BalanceAttrs.HEIGHT, i] = (
                LEG_TAR_H - leg.body.avg_leg_h)

            if balance_diffs[BalanceAttrs.IMP_INV][i] > 0.33:
                balance_cache[i] += l.balance_pid.eval(
                    balance_cache[i], balance_cache[i] + base_part[i])
                balance_cache[i] += l.touch_pid.eval(
                    balance_cache[i], balance_cache[i])
            else:
                balance_cache[i] += l.balance_pid.eval(
                    balance_cache[i], balance_cache[i] + base_part[i])
                balance_cache[i] += l.touch_pid.eval(
                    balance_cache[i], balance_cache[i] + balance_diffs[BalanceAttrs.TOUCH][i] + balance_diffs[BalanceAttrs.DROP][i])

    balance_manager[leg.idx] = 1

    return balance_cache[leg.idx]
