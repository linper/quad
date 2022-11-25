
import pybullet as p
# from main import ev_prox_par
import numpy as np
from consts import *
from datetime import datetime
import os
import math
from numba import njit
import matplotlib.pyplot as plt
from interp import line_cof, get_mv_dot_product, get_2x2_rotation_matrix_from_angle, get_vectors_sine, get_vectors_cosine, get_vectors_angle, ellipse_line_intersect, ellipse_point_inside, vector_projection, strech_vector_to, xor, point_to_sect, area, is_inside, centroid_of_polygon, make_cw

import wrapper as w

ev_prox_par = ev_avg_par = ev_std_par = ev_edge_par = ev_ap_dir_par = ev_cp_dir_par = ev_sup_tri_par = ev_junct_par = ev_com_bal_par = None

SAVE_DIR = "./save"

COF_MAX = 9
COFS = np.zeros(COF_MAX, dtype=float)


def cofs_set():
    global COFS

    COF_PROX = 0
    COF_STD = 1
    COF_AVG = 2
    COF_EDGE = 3
    COF_ADIR = 4
    COF_CDIR = 5
    COF_STRI = 6
    COF_JUNCT = 7
    COF_COM_BAL = 8

    COFS[COF_PROX] = p.readUserDebugParameter(ev_prox_par)
    COFS[COF_STD] = p.readUserDebugParameter(ev_std_par)
    COFS[COF_AVG] = p.readUserDebugParameter(ev_avg_par)
    COFS[COF_EDGE] = p.readUserDebugParameter(ev_edge_par)
    COFS[COF_ADIR] = p.readUserDebugParameter(ev_ap_dir_par)
    COFS[COF_CDIR] = p.readUserDebugParameter(ev_cp_dir_par)
    COFS[COF_STRI] = p.readUserDebugParameter(ev_sup_tri_par)
    COFS[COF_JUNCT] = p.readUserDebugParameter(ev_junct_par)
    COFS[COF_COM_BAL] = p.readUserDebugParameter(ev_com_bal_par)


@njit
def cofs_get():
    return COFS


def guess_start_pos(off, d, ell_a, ell_b, n):
    k, b = line_cof(0.0, 0.0, d[0], d[1])
    _, p1, p2 = ellipse_line_intersect(ell_a, ell_b, k, b)
    p1 = p1 + off
    p2 = p2 + off
    pts = np.linspace(p1, p2, n + 2)[1:-1]

    return pts


@njit
def eval_sequence(pts, def_pts, d, ell_a, ell_b):
    edge_retr_vals = []
    dists_left = []
    max_d = 0.0

    for i in range(pts.shape[0]):
        # leg offset from default position
        l_off = pts[i] - def_pts[i]
        # checking if leg is inside area restricted by ellipse
        inside, ins_val = ellipse_point_inside(
            ell_a, ell_b, l_off[0], l_off[1])
        if not inside:
            # unlikely
            edge_retr_vals.append(1.0)
        else:
            # leg offset from default position + direction vector
            l_off_mod = l_off + d
            # gets line coafitients of leg movenet according to its default position
            k, b = line_cof(l_off[0], l_off[1], l_off_mod[0], l_off_mod[1])
            # gets points where leg will cross ellipse restricted area if direction is maintained
            _, p1, p2 = ellipse_line_intersect(
                ell_a, ell_b, k, b)

            # checking if perpendicular vectors pointing in same direction
            if not xor((p1 - p2)[0] > 0, d[0] > 0):
                pf = p1
            else:
                pf = p2

            # how much leg point is away from middle
            erv = 1 - (1 - ins_val*ins_val)**0.5
            edge_retr_vals.append(erv)

            # getting distance till restricted area
            dist_left = np.linalg.norm(l_off - pf)
            dists_left.append(dist_left)

            # getin maximum distance from p1 to p2 among legs
            full_d = np.linalg.norm(p1 - p2)
            if max_d < full_d:
                max_d = full_d

    edge_part = np.sum(np.array(edge_retr_vals))

    if len(dists_left) <= 1:
        print(f"len of dists_left was:{len(dists_left)}")
        return 1.0, 1.0, edge_part

    # adding outer values for widest p1/p2 pair
    dists_left.append(0.0)
    dists_left.append(max_d)

    dists_left.sort()

    dl_arr = np.array(dists_left)
    difs = dl_arr[2:-1] - dl_arr[1:-2]
    difs_mod = dl_arr[1:] - dl_arr[:-1]

    # inverted diffs average to elipse max half-axis relation
    avg_part = 2 - (np.average(difs) / (max_d / len(difs)))
    # STD + some constant to encourage legs to move from default position
    std_part = np.std(difs_mod) / \
        np.average(difs_mod) if np.average(difs_mod) != 0.0 else 0.0

    return avg_part, std_part, edge_part


@njit
def eval_min_prox(pts, idx):
    k = -30
    r = 0.006
    p0 = pts[idx]
    min_val = np.min(np.array([np.linalg.norm(p - p0)
                               for i, p in enumerate(pts) if i != idx]))
    val = min(max(k * min_val - k * r + 1, 0.0), 1.0)
    return val


@njit
def eval_support_shape(pts, idx):
    max_ps_coaf = 16
    cw = [0, 1, 3, 2]

    # clockwise points adjusted by default leg excentricity
    cw_pts = [pts[cw[(idx+i) % 4]] / np.array([get_leg_exc(), 1.0])
              for i in range(-len(cw), 0)]
    perimeter = np.sum(
        np.array([np.linalg.norm(cw_pts[i] - cw_pts[i-1]) for i in range(4)]))
    pair1 = (area(cw_pts[0], cw_pts[1], cw_pts[2]),
             area(cw_pts[2], cw_pts[3], cw_pts[0]))
    pair2 = (area(cw_pts[1], cw_pts[2], cw_pts[3]),
             area(cw_pts[3], cw_pts[0], cw_pts[1]))
    surface = (pair1[0] + pair1[1] + pair2[0] + pair2[1]) / 2

    val = 1 - ((max_ps_coaf * surface) / (perimeter ** 2))
    return val


@njit
def eval_avg_move_dir(pts, direction, R):
    d = strech_vector_to(direction, R)
    avg_pos_diff = np.sum(pts, axis=0)

    apd_vec = strech_vector_to(d, vector_projection(avg_pos_diff, d))

    apd_sign = -1 if xor(apd_vec[0] > 0, d[0] > 0) else 1

    ap_part = 1 - (R + (apd_sign * np.linalg.norm(apd_vec) / (2 * R)))

    # apa = get_vectors_angle(avg_pos_diff, direction)
    # ap_part *= apa

    return ap_part


# @njit
def loss(pts, def_pts, idx, d, ell_a, ell_b):
    COF_PROX = 0
    COF_STD = 1
    COF_AVG = 2
    COF_EDGE = 3
    COF_ADIR = 4
    COF_CDIR = 5
    COF_STRI = 6
    COF_JUNCT = 7
    COF_COM_BAL = 8

    cofs = cofs_get()

    val = 0

    prox_part = eval_min_prox(pts, idx)
    val += cofs[COF_PROX] * prox_part

    avg_part, std_part, edge_part = eval_sequence(
        pts, def_pts, d, ell_a, ell_b)
    val += cofs[COF_AVG] * avg_part
    val += cofs[COF_STD] * std_part
    val += cofs[COF_EDGE] * edge_part

    apd_part = eval_avg_move_dir(pts, d, max(ell_a, ell_b))
    val += cofs[COF_ADIR] * apd_part

    sup_shp_part = eval_support_shape(pts, idx)
    val += cofs[COF_STRI] * sup_shp_part

    # junct_part = self.eval_junctions()
    # val += junct_c * junct_part

    # com_bal_part = self.eval_com_balance()
    # val += com_bal_c * com_bal_part

    # cpd_part = self.eval_com_move_dir()
    # val += cp_dir_c * cpd_part

    # print(f"part:{[prox_c * prox_part, std_c * std_part, avg_c * avg_part, ap_dir_c * apd_part, cp_dir_c * cpd_part, sup_tri_c * sup_shp_part]}")

    return val


class Stance:
    def __init__(self, pos, def_pos, direction, com):
        self.next = None
        self.prev = None
        self.pts: np.ndarray = pos.copy()
        self.orig_pts: np.ndarray = pos.copy()
        self.def_pts: np.ndarray = def_pos.copy()
        self.com: np.ndarray = com.copy()
        self.direction: np.ndarray = direction.copy()
        self.pidx: int = -1
        self.min_loss: float = -1.0
        self.ell_a: float = LEG_ELLIPSE_A
        self.ell_b: float = LEG_ELLIPSE_B

    def clone(self):
        return Stance(self.orig_pts.copy(), self.def_pts.copy(), self.direction.copy(), self.com.copy())


    def grad(self, spts, pts, def_pts, idx, d, ell_a, ell_b):
        delta = 1e-5
        default = np.zeros(len(spts), dtype=float)
        gradient = np.zeros(shape=(len(spts), pts.shape[1]), dtype=float)
        c_pts = pts.copy()
        for i in range(len(spts)):
            c_pts = pts.copy()
            c_pts[idx] = spts[i]
            default[i] = loss(c_pts, def_pts, idx, d, ell_a, ell_b)
            for j in range(pts.shape[1]):
                c_pts[idx][j] += delta
                gradient[i][j] = (
                    loss(c_pts, def_pts, idx, d, ell_a, ell_b) - default[i]) / delta

                # restoring
                c_pts[idx][j] -= delta

        return gradient, default

    def get_optimal_step(self, idx, pts, def_pts, d, ell_a, ell_b):
        max_iter = 25
        epsilon = 1e-3
        alpha = 0.01
        i = 0
        n_start_pts = 6

        losses = []
        last_loss = np.array([float("inf") for _ in range(n_start_pts)])

        spts = guess_start_pos(def_pts[idx], d, ell_a, ell_b, n_start_pts)
        paths = [spts.copy()]
        # print(spts)
        while i < max_iter:
            g, l = self.grad(spts, pts, def_pts, idx, d, ell_a, ell_b)
            spts = spts - alpha * g
            paths.append(spts.copy())
            losses.append(l)
            d_loss = last_loss - l
            last_loss = l
            i += 1
            if epsilon * alpha > np.max(d_loss):
                break

        # return paths, losses

        # this gets best point and its loss
        min_loss_idx = 0
        min_loss = last_loss[min_loss_idx]
        for i, l in enumerate(last_loss):
            if l < min_loss:
                min_loss_idx = i
                min_loss = l
        
        return paths[min_loss_idx][-1], min_loss


    def optimize(self, n):
        en = self
        # iterating to last node
        while en.next is not None:
            en = en.next

        for _ in range(n):
            print(f"pts:{en.pts} com:{en.com}")
            opt_vals = [en.get_optimal_step(i, en.pts, en.def_pts, en.direction, en.ell_a, en.ell_b) for i in range(4)]

            # Gets best point and loss of best leg
            min_loss_idx = 0
            min_pair = opt_vals[min_loss_idx]
            for i, pair in enumerate(opt_vals):
                if pair[1] < min_pair[1]:
                    min_loss_idx = i
                    min_pair = pair

            # Seting which leg should be moved
            en.pidx = min_loss_idx
            en.min_loss = min_pair[1]
            en.pts[en.pidx] = min_pair[0]
            print(f"opt point:{min_pair[0]} loss:{min_pair[1]} pidx:{min_loss_idx}")
            
            # Creating next stance
            com = centroid_of_polygon(make_cw(en.pts)) 
            # new_stc = Stance(en.pts, en.def_pts, en.direction, com)
            new_stc = Stance(en.pts - com, en.def_pts, en.direction, np.zeros(2, dtype=float))

            # Linking stances
            en.next = new_stc
            new_stc.prev = en

            en = new_stc
    

    def plot(self):
        INC = 0.01
        # n_opt_stances = 3
        n_opt_stances = 10
        stance_id = 0
        # move_com = False
        # move_com = True

        cofs_set()

        self.optimize(n_opt_stances)

        en = self
        
        while en is not None and en.pidx != -1:
            print(f"{en.orig_pts[en.pidx]} ({en.pidx})=> {en.pts[en.pidx]}")
            opt_pos = en.pts[en.pidx].copy()
            st_pos = en.def_pts[en.pidx].copy()
            # com = en.com.copy()
            heatmap = np.zeros((64, 64), dtype=float)
            shp = heatmap.shape
            for i, y in enumerate(range(shp[0] // 2 - shp[0], shp[0] // 2)):
                for j, x in enumerate(range(shp[1] // 2 - shp[1], shp[1] // 2)):
                    en.pts[en.pidx] = st_pos + INC * np.array([x, y])
                    heatmap[i, j] = en.eval_wrap()

            c = plt.imshow(heatmap.T, cmap="hot", interpolation="nearest")
            plt.colorbar(c)

            en.pts[en.pidx] = opt_pos
            
            dp_mod = en.def_pts - st_pos
            op_mod = en.orig_pts - st_pos
            com = en.com - st_pos

            print(f"{en.orig_pts[en.pidx]} ({en.pidx})==> {en.pts[en.pidx]}")
            optimized_pt = en.pts[en.pidx] - st_pos

            min_x = 0
            min_y = 0
            min_val = 1000.0
            for y in range(shp[0]):
                for x in range(shp[1]):
                    if heatmap[y, x] < min_val:
                        min_x = x
                        min_y = y
                        min_val = heatmap[y, x]
            
            plt.scatter(dp_mod[:, 1] / INC + shp[0] / 2,
                        dp_mod[:, 0] / INC + shp[1] / 2, color="blue", label="default")
            plt.scatter(op_mod[:, 1] / INC + shp[0] / 2,
                        op_mod[:, 0] / INC + shp[1] / 2, color="green", label="original")
            plt.scatter(min_y, min_x, color="red",
                        label=f"min IT:{round(min_val, 4)}")

            plt.scatter(optimized_pt[1] / INC + shp[0] / 2,
                        optimized_pt[0] / INC + shp[1] / 2, color="orange",
                        label=f"min GD:{round(en.min_loss, 4)}")
            plt.scatter(com[1] / INC + shp[0] / 2,
                        com[0] / INC + shp[1] / 2, color="black",
                        label=f"com")

            plt.legend()
            plt.title(f"{datetime.now()}_{stance_id}")
            plt.savefig(f"{SAVE_DIR}/{datetime.now()}_{stance_id}.png")
            plt.show()

            en = en.next
            stance_id += 1



    # def plot(self):
        # n_opt = 1
        # move_com = False
        # # move_com = True

        # cofs_set()
        # INC = 0.01
        # orig_pos = self.pts[self.pidx].copy()
        # com = self.com.copy()
        # heatmap = np.zeros((64, 64), dtype=float)
        # for i, y in enumerate(range(heatmap.shape[0] // 2 - heatmap.shape[0], heatmap.shape[0] // 2)):
            # for j, x in enumerate(range(heatmap.shape[1] // 2 - heatmap.shape[1], heatmap.shape[1] // 2)):
                # if move_com:
                    # self.com = com + INC * np.array([x, y])
                # else:
                    # self.pts[self.pidx] = orig_pos + INC * np.array([x, y])

                # heatmap[i, j] = self.eval_wrap()

        # c = plt.imshow(heatmap.T, cmap="hot", interpolation="nearest")
        # plt.colorbar(c)

        # # print(guess_start_pos(self.def_pts[self.pidx], self.direction, self.ell_a, self.ell_b, 4))
        # self.pts[self.pidx] = orig_pos
        # paths, losses = self.optimize(
            # self.pts, self.def_pts, self.pidx, self.direction, self.ell_a, self.ell_b)

        # min_x = 0
        # min_y = 0
        # min_val = 1000.0
        # for y in range(heatmap.shape[0]):
            # for x in range(heatmap.shape[1]):
                # if heatmap[y, x] < min_val:
                    # min_x = x
                    # min_y = y
                    # min_val = heatmap[y, x]

        # if move_com:
            # dp_mod = self.def_pts
            # op_mod = self.orig_pts
        # else:
            # dp_mod = self.def_pts - orig_pos
            # op_mod = self.orig_pts - orig_pos

        # plot_path = np.array(
            # [(i - orig_pos) / INC + np.array(heatmap.shape, dtype=float) / 2 for i in paths])
        # # print(f"plot_path:{plot_path}")
        # # print(f"losses:{losses}")

        # seg_len = len(plot_path[0])
        # paths_lst = [[[], []] for _ in range(seg_len)]
        # losses_lst = [[] for _ in range(seg_len)]
        # for j, seg in enumerate(plot_path):
            # for i in range(seg_len):
                # paths_lst[i][0].append(seg[i][0])
                # paths_lst[i][1].append(seg[i][1])

        # for j, ls in enumerate(losses):
            # for i in range(seg_len):
                # losses_lst[i].append(ls[i])

        # min_losses = np.array([np.min(np.array(l)) for l in losses_lst])
        # min_idx = [i for i, j in enumerate(min_losses) if j == np.min(min_losses)][0]
        # # print(f"paths_lst:{paths_lst}")
        # # print(f"losses_lst:{losses_lst}")
        # # print()

        # plt.scatter(dp_mod[:, 1] / INC + heatmap.shape[0] / 2,
                    # dp_mod[:, 0] / INC + heatmap.shape[1] / 2, color="blue", label="default")
        # plt.scatter(op_mod[:, 1] / INC + heatmap.shape[0] / 2,
                    # op_mod[:, 0] / INC + heatmap.shape[1] / 2, color="green", label="original")
        # plt.scatter(min_y, min_x, color="red",
                    # label=f"min IT:{round(min_val, 4)}")

        # plt.scatter(paths_lst[min_idx][1][-1], paths_lst[min_idx][0][-1], color="orange",
                    # label=f"min GD:{round(min_losses[min_idx], 4)}")
        # for i, pth in enumerate(paths_lst):
            # plt.plot(pth[1], pth[0], label="_nolegend_")
                     # # label=f"path:{i}:{round(min_losses[i], 4)}")

        # plt.legend()
        # plt.savefig(f"{SAVE_DIR}/{datetime.now()}.png")
        # plt.show()

        # # for l in losses_lst:
            # # plt.plot(l)
        # # plt.show()

    def eval_com_move_dir(self):
        R = max(self.ell_a, self.ell_b)
        d = strech_vector_to(self.direction, R)
        # avg_pos_diff = np.sum(self.pts, axis=0)
        com_pos_diff = self.com - self.com

        # apd_vec = strech_vector_to(d, vector_projection(avg_pos_diff, d))
        cpd_vec = strech_vector_to(d, vector_projection(com_pos_diff, d))

        # apa = get_vectors_angle(avg_pos_diff, self.direction)
        cpa = get_vectors_angle(com_pos_diff, self.direction)

        # apd_sign = -1 if xor(apd_vec[0] > 0, d[0] > 0) else 1
        cpd_sign = -1 if xor(cpd_vec[0] > 0, d[0] > 0) else 1

        # ap_part = 1 - (R + (apd_sign * np.linalg.norm(apd_vec) / (2 * R)))
        cp_part = 1 - (R + (cpd_sign * np.linalg.norm(cpd_vec) / (2 * R)))

        # ap_part *= apa
        cp_part *= cpa

        return cp_part

    def eval_junctions(self):
        R = 0.012
        val = 0.0

        for i in range(4):
            pt1 = self.pts[i]
            for j in range(i + 1, 4):
                pt2 = self.pts[j]
                dist = point_to_sect(self.com, pt1, pt2)
                if dist == -1.0:
                    continue
                # 0.5 because same section gets counted twice
                val = max(val, 0.5 * (1 - (dist / R)) if dist < R else 0.0)

        return val

    def eval_com_balance(self):
        leg_ch_thr = 0.002
        leg_ch = [np.linalg.norm(a) for a in self.pts - self.orig_pts]
        max_leg_ch = max(leg_ch)
        # indices for balanced legs
        bids = list(range(4))
        for i, v in enumerate(leg_ch):
            if round(v - max_leg_ch, 4) == 0:
                bids.pop(i)
                break

        P = self.pts

        # com is inside of area limited by 4 legs before
        bal4be = (is_inside(self.com, P[0], P[1], P[3]) or
                  is_inside(self.com, P[2], P[3], P[0]))
        # com is inside of area limited by 4 legs after
        bal4af = (is_inside(self.com, P[2], P[3], P[0]) or
                  is_inside(self.com, P[0], P[1], P[3]))

        bal4 = bal4af and bal4be

        # com is inside of area limited by 3 grounded legs
        bal_gr = is_inside(self.com, P[bids[0]], P[bids[1]], P[bids[2]]) and \
            is_inside(self.com, P[bids[0]], P[bids[1]], P[bids[2]])

        # was leg moved
        leg_mov = max_leg_ch > leg_ch_thr

        if leg_mov and bal_gr:
            val = 0.0
        elif leg_mov and bal4:
            val = 0.5
        elif bal4:
            # no leg was lifted
            val = 0.0
        elif bal4af or bal4be:
            # no leg was lifted
            val = 0.75
        else:
            val = 1.0

        return val

    def eval_wrap(self, dummy_pt=None, dummy_idx=-1):
        is_dummy = dummy_pt is not None and dummy_idx != -1
        pts_save = self.pts.copy()

        if is_dummy:
            self.pts[dummy_idx] = dummy_pt

        res = loss(self.pts, self.def_pts, self.pidx,
                   self.direction, self.ell_a, self.ell_b)

        if is_dummy:
            self.pts = pts_save

        return res

        prox_c = p.readUserDebugParameter(ev_prox_par)
        std_c = p.readUserDebugParameter(ev_std_par)
        avg_c = p.readUserDebugParameter(ev_avg_par)
        edge_c = p.readUserDebugParameter(ev_edge_par)
        ap_dir_c = p.readUserDebugParameter(ev_ap_dir_par)
        cp_dir_c = p.readUserDebugParameter(ev_cp_dir_par)
        sup_tri_c = p.readUserDebugParameter(ev_sup_tri_par)
        junct_c = p.readUserDebugParameter(ev_junct_par)
        com_bal_c = p.readUserDebugParameter(ev_com_bal_par)

        # print(f"coafs:{[prox_c, std_c, avg_c, ap_dir_c, cp_dir_c, sup_tri_c]}")

        val = 0

        prox_part = eval_min_prox(self.pts, self.pidx)
        val += prox_c * prox_part

        avg_part, std_part, edge_part = eval_sequence(
            self.pts, self.def_pts, self.direction, self.ell_a, self.ell_b)
        val += avg_c * avg_part
        val += std_c * std_part
        val += edge_c * edge_part

        apd_part = eval_avg_move_dir(
            self.pts, self.direction, max(self.ell_a, self.ell_b))
        val += ap_dir_c * apd_part

        sup_shp_part = eval_support_shape(self.pts, self.pidx)
        val += sup_tri_c * sup_shp_part

        # junct_part = self.eval_junctions()
        # val += junct_c * junct_part

        # com_bal_part = self.eval_com_balance()
        # val += com_bal_c * com_bal_part

        # cpd_part = self.eval_com_move_dir()
        # val += cp_dir_c * cpd_part

        # print(f"part:{[prox_c * prox_part, std_c * std_part, avg_c * avg_part, ap_dir_c * apd_part, cp_dir_c * cpd_part, sup_tri_c * sup_shp_part]}")

        return val


def clear_saves():
    if not os.path.exists(SAVE_DIR):
        os.mkdir(SAVE_DIR)
        return

    for f in os.listdir(SAVE_DIR):
        path = os.path.join(SAVE_DIR, f)
        try:
            os.remove(path)
        except OSError as error:
            print(error)
            print("File path can not be removed")

    return


def init_debug_params():
    global ev_prox_par, ev_avg_par, ev_std_par, ev_edge_par, ev_ap_dir_par, ev_cp_dir_par, ev_sup_tri_par, ev_junct_par, ev_com_bal_par

    ev_prox_par = p.addUserDebugParameter("prox", 0.0, 1., EV_MIN_PROX_C)
    ev_avg_par = p.addUserDebugParameter("avg", 0.0, 1., EV_AVG_C)
    ev_std_par = p.addUserDebugParameter("std", 0.0, 1., EV_STD_C)
    ev_edge_par = p.addUserDebugParameter("edge", 0.0, 1., EV_EDGE_C)
    ev_ap_dir_par = p.addUserDebugParameter("ap_dir", 0.0, 1., EV_AP_DIR_C)
    ev_cp_dir_par = p.addUserDebugParameter("cp_dir", 0.0, 1., EV_CP_DIR_C)
    ev_sup_tri_par = p.addUserDebugParameter(
        "sup_tri_dir", 0.0, 1., EV_SUP_TRI_C)
    ev_junct_par = p.addUserDebugParameter(
        "junction", 0.0, 1., EV_JUNCT_C)
    ev_com_bal_par = p.addUserDebugParameter(
        "com_balance", 0.0, 1., EV_COM_BAL_C)
