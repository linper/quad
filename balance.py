
from enum import IntEnum
import functools

from plan import *


class BalanceAttrs(IntEnum):
    HIGHT = 0
    LEAN = 1
    IMP = 2
    IMP_INV = 3
    TOUCH = 4
    MAX = 5


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
    force_r_mat = get_rotation_matrix_from_two_vectors(
        np.array([0.0, 0.0, -1.0]), leg.body.sens_info.base_force_vector)
    leg_pos_mod = np.dot(leg.position, force_r_mat)
    leg_dif = leg_pos_mod - leg.position

    return leg_dif[2]


def get_touch_diff(leg):
    hits, adj = leg.check_damp()

    if hits and ADJUST_SENS < abs(adj):
        return 0.4 * adj
    elif not hits:
        return -0.4 * T_RAD
    else:
        return 0.0


balance_cache = np.zeros(4, dtype=float)
balance_manager = np.zeros(4)


def get_balance_diff(leg):
    global balance_cache, balance_manager

    lean_cof = 0.012
    hight_cof = 0.06
    rebalance_cof = 0.7
    hight_reb_act_cof = 0.3

    if balance_manager[leg.idx] == 1:
        balance_manager = np.zeros(4)

        balance_diffs = np.zeros((BalanceAttrs.MAX, 4), dtype=float)

        for i, l in enumerate(leg.body.legs):
            balance_diffs[BalanceAttrs.TOUCH, i] = get_touch_diff(l)
            balance_diffs[BalanceAttrs.LEAN, i] = get_lean_diff(l)
            balance_diffs[BalanceAttrs.IMP,
                          i], balance_diffs[BalanceAttrs.IMP_INV, i] = get_importance(l)
            balance_diffs[BalanceAttrs.HIGHT, i] = (
                LEG_TAR_H - leg.body.avg_leg_h)

        stage_1 = lean_cof * balance_diffs[BalanceAttrs.LEAN] + \
            balance_diffs[BalanceAttrs.TOUCH] * \
            balance_diffs[BalanceAttrs.IMP_INV]

        leftover_part = (stage_1 + hight_cof *
                         balance_diffs[BalanceAttrs.HIGHT]) / stage_1
        print(leftover_part)

        while True in (abs(b) < hight_reb_act_cof for b in leftover_part):
            hight_cof = hight_cof * rebalance_cof
            leftover_part = (stage_1 + hight_cof *
                             balance_diffs[BalanceAttrs.HIGHT]) / stage_1
            print("rebalance")

        print(leftover_part)

        balance_cache = stage_1 + hight_cof * balance_diffs[BalanceAttrs.HIGHT]

    balance_manager[leg.idx] = 1

    return balance_cache[leg.idx]
