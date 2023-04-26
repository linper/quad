
from enum import IntEnum
from pidc import PIDC2
import functools

import numpy as np

from plan import *
from consts import MAX_WALK_H, MIN_WALK_H
from numba import njit


class BalanceAttrs(IntEnum):
    IMP = 0
    TOUCH = 1
    DROP = 2
    MAX = 3


def get_importance(leg):
    x_pos = [l.position[0] for l in leg.body.legs if l.do_balance]
    x_pos.append(leg.body.sens_info.t_force_info.pos[0])
    min_x_pos = functools.reduce(lambda a, b: a if a < b else b, x_pos)
    max_x_pos = functools.reduce(lambda a, b: a if a > b else b, x_pos)
    x_pos_diff = max_x_pos - min_x_pos
    x_lever = abs(
        leg.position[0] - leg.body.sens_info.t_force_info.pos[0]) / x_pos_diff

    y_pos = [l.position[1] for l in leg.body.legs if l.do_balance]
    y_pos.append(leg.body.sens_info.t_force_info.pos[1])
    min_y_pos = functools.reduce(lambda a, b: a if a < b else b, y_pos)
    max_y_pos = functools.reduce(lambda a, b: a if a > b else b, y_pos)
    y_pos_diff = max_y_pos - min_y_pos
    y_lever = abs(
        leg.position[1] - leg.body.sens_info.t_force_info.pos[1]) / y_pos_diff

    leg_imp_share = x_lever * y_lever

    return leg_imp_share


def get_lean_diff(leg):
    if leg.body.sens_info.damp[leg.idx] < SOFT_HIT_THR:
        return leg.body.sens_info.avg_leg_h - leg.position[2]

    force_r_mat = get_rotation_matrix_from_two_vectors(
        np.array([0.0, 0.0, -1.0]), leg.body.sens_info.base_force_vector)
    leg_pos_mod = get_dot_product(leg.position, force_r_mat)
    leg_dif = leg_pos_mod - leg.position

    return leg_dif[2]


def get_balance_base(q, cof):
    res = np.zeros(4, dtype=float)

    bfo_mat = get_4x4_from_3x3_mat(q.sens_info.base_frame_orientation_matrix
                                   )
    height_mat = np.identity(4, dtype=float)

    # making shift matrix in z axis
    height_mat[2, 3] = LEG_TAR_H - q.sens_info.avg_leg_h

    for i, l in enumerate(q.legs):
        if q.sens_info.damp[l.idx] < SOFT_HIT_THR:
            res[i] = cof * (l.body.sens_info.avg_leg_h - l.position[2])
            continue

        leg_pos_mod = np.ones(4, dtype=float)
        leg_pos_mod[:3] = l.position
        leg_pos_mod = get_mv_dot_product(bfo_mat, leg_pos_mod)
        leg_pos_mod = get_mv_dot_product(height_mat, leg_pos_mod)
        # leg_pos_mod = bfo_mat.dot(leg_pos_mod)
        # leg_pos_mod = height_mat.dot(leg_pos_mod)

        leg_dif = leg_pos_mod[:3] - l.position

        res[i] = (leg_dif * cof)[2]

    return res


def get_touch_diff(leg):
    _, s_hits, adj = leg.check_damp()
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

    # TOUCH_COF = 1.0
    TOUCH_COF = 0.2

    if balance_manager[leg.idx] == 1:
        balance_manager = np.zeros(4)
        balance_cache = np.zeros(4, dtype=float)

        balance_diffs = np.zeros((BalanceAttrs.MAX, 4), dtype=float)

        base_part = get_balance_base(leg.body, 1.0)

        for i, l in enumerate(leg.body.legs):
            if not l.do_balance:
                continue

            balance_diffs[BalanceAttrs.DROP,
                          i], balance_diffs[BalanceAttrs.TOUCH, i] = get_touch_diff(l)
            balance_diffs[BalanceAttrs.IMP, i] = get_importance(l)

            balance_cache[i] = l.balance_pid.eval(
                0.0, base_part[i])

            if balance_diffs[BalanceAttrs.IMP][i] > 0.33:
                balance_cache[i] += l.touch_pid.eval(
                    balance_cache[i], balance_cache[i])
            else:
                balance_cache[i] += l.touch_pid.eval(
                    balance_cache[i], balance_cache[i] + TOUCH_COF * balance_diffs[BalanceAttrs.TOUCH][i] + balance_diffs[BalanceAttrs.DROP][i])

        print(f"drop:{balance_diffs[BalanceAttrs.DROP]}")
        print(f"touch:{balance_diffs[BalanceAttrs.TOUCH]}")
        print(f"imp:{balance_diffs[BalanceAttrs.IMP]}")
        print(f"final:{balance_cache}")

    balance_manager[leg.idx] = 1

    return np.array([0.0, 0.0, balance_cache[leg.idx]])


def get_walk_height(step_dist, abs_adj_h):
    # acording to 0 < abs_adj_h < 0.1
    aah_part = (-0.00088512 / (abs_adj_h + 0.0081818)) - 0.091818
    sd_part = (-0.018857 / (step_dist + 0.17143)) - 0.09

    val = max(aah_part, sd_part)
    res = min(MAX_WALK_H, max(MIN_WALK_H, val))
    print(f"sd:{round(step_dist, 3)} => {round(sd_part, 3)} aah:{round(abs_adj_h, 3)} => {round(aah_part, 3)} res:{round(res, 3)}")
    return res
