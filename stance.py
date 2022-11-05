
import pybullet as p
# from main import ev_prox_par
import numpy as np
from consts import *
from datetime import datetime
import os
import math
import matplotlib.pyplot as plt
from interp import line_cof, get_mv_dot_product, get_2x2_rotation_matrix_from_angle, get_vectors_sine, get_vectors_cosine, get_vectors_angle, ellipse_line_intersect, ellipse_point_inside, vector_projection, strech_vector_to, xor

ev_prox_par = ev_avg_par = ev_std_par = ev_edge_par = ev_ap_dir_par = ev_cp_dir_par = ev_sup_tri_par = None

SAVE_DIR = "./save"


class Stance:
    def __init__(self, q, lg, target):
        self.q = q
        self.pts: np.ndarray = np.zeros((4, 2), dtype=float)
        self.def_pts: np.ndarray = np.zeros((4, 2), dtype=float)
        # self.orig_com: np.ndarray = np.sum(self.pts, axis=0)
        self.orig_com: np.ndarray = np.zeros(2, dtype=float)
        # TODO: hardcoded 'com' ffor testing <29-10-22, yourname> #
        self.com: np.ndarray = self.orig_com + target.copy()
        self.direction: np.ndarray = target.copy()
        # self.com: np.ndarray = self.orig_com.copy()
        self.pidx = lg.idx
        self.ell_a = lg.ellipse_a
        self.ell_b = lg.ellipse_b

        for i, l in enumerate(q.legs):
            self.pts[i] = l.position[:2]
            self.def_pts[i] = l.def_pos[:2]

        self.orig_pts = self.pts.copy()

    def optimize(self):
        pass

    def plot(self):
        INC = 0.01
        orig_pos = self.pts[self.pidx].copy()
        heatmap = np.zeros((64, 64), dtype=float)
        # self.pts[self.pidx] = orig_pos + 0.01 * np.array([-30, -30])
        # heatmap[0, 0] = self.eval()
        for i, y in enumerate(range(heatmap.shape[0] // 2 - heatmap.shape[0], heatmap.shape[0] // 2)):
            for j, x in enumerate(range(heatmap.shape[1] // 2 - heatmap.shape[1], heatmap.shape[1] // 2)):
                self.pts[self.pidx] = orig_pos + INC * np.array([x, y])
                heatmap[i, j] = self.eval()

        c = plt.imshow(heatmap.T, cmap="hot", interpolation="nearest")
        plt.colorbar(c)

        min_x = 0
        min_y = 0
        min_val = 1000.0
        for y in range(heatmap.shape[0]):
            for x in range(heatmap.shape[1]):
                if heatmap[y, x] < min_val:
                    min_x = x
                    min_y = y
                    min_val = heatmap[y, x]

        # dp_mod = self.def_pts - orig_pos
        # op_mod = self.orig_pts - orig_pos
        dp_mod = self.def_pts - orig_pos
        op_mod = self.orig_pts - orig_pos
        plt.scatter(dp_mod[:, 1] / INC + heatmap.shape[0] / 2,
                    dp_mod[:, 0] / INC + heatmap.shape[1] / 2, color="blue", label="default")
        plt.scatter(op_mod[:, 1] / INC + heatmap.shape[0] / 2,
                    op_mod[:, 0] / INC + heatmap.shape[1] / 2, color="green", label="original")
        plt.scatter(min_y, min_x, color="red",
                    label=f"min:{round(min_val, 4)}")

        plt.legend()
        plt.savefig(f"{SAVE_DIR}/{datetime.now()}.png")
        plt.show()

    def eval_sequence(self):
        if np.array_equal(self.orig_com, self.com):
            return 0.0, 0.0, 0.0

        edge_retr_vals = []
        dists_left = []
        # epsilon = 1.0
        epsilon = 0.6
        max_d = 0.0

        d = self.direction
        # d = self.orig_com - self.com

        for i in range(self.pts.shape[0]):
            # leg offset from default position
            l_off = self.pts[i] - self.def_pts[i]
            # checking if leg is inside area restricted by ellipse
            inside, ins_val = ellipse_point_inside(
                self.ell_a, self.ell_b, l_off[0], l_off[1])
            if not inside:
                # unlikely
                edge_retr_vals.append(1.0)
                # reducing slope to 0.2
                # TODO: delete slope later <30-10-22, yourname> #
                # edge_retr_vals.append((0.2*(ins_val-1)) + 1)
            else:
                # leg offset from default position + direction vector
                l_off_mod = l_off + d
                # gets line coafitients of leg movenet according to its default position
                k, b = line_cof(l_off[0], l_off[1], l_off_mod[0], l_off_mod[1])
                # gets points where leg will cross ellipse restricted area if direction is maintained
                _, p1, p2 = ellipse_line_intersect(
                    self.ell_a, self.ell_b, k, b)

                # TODO: Maybe use xor <01-11-22, yourname> #
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
            return 1.0, 1.0, edge_part

        # adding outer values for widest p1/p2 pair
        dists_left.append(0.0)
        dists_left.append(max_d)

        dists_left.sort()

        difs = np.array(
            [p2 - p1 for p1, p2 in zip(dists_left[:-2], dists_left[2:])])
        difs_mod = np.array(
            [p2 - p1 for p1, p2 in zip(dists_left[:-1], dists_left[1:])])

        # inverted diffs average to elipse max half-axis relation
        avg_part = 1 - (np.average(difs) /
                        (2 * max(self.ell_a, self.ell_b) / 3))
        # STD + some constant to encourage legs to move from default position
        # TODO: optimize epsilon <30-10-22, yourname> #
        # std_part = np.std(difs) + epsilon
        # std_part = np.std(difs)

        std_part = np.std(difs_mod) / \
            np.average(difs_mod) if np.average(difs_mod) != 0.0 else 0.0

        # std_part = np.std(difs) / np.average(difs)
        # seq_part = std_part
        # seq_part = avg_part
        # seq_part = avg_part * std_part

        # print(f"mat:{rot_mat}\n before:{self.pts}\n after:{rotated}")
        # plt.scatter(self.pts[:, 0], self.pts[:, 1], color="red")
        # plt.scatter(rotated[:, 0], rotated[:, 1], color="blue")
        # plt.axis("equal")
        # plt.show()

        return avg_part, std_part, edge_part

    def eval_min_prox(self):
        k = -30
        r = 0.006
        p0 = self.pts[self.pidx]
        min_val = np.min(np.array([np.linalg.norm(p - p0)
                                   for i, p in enumerate(self.pts) if i != self.pidx]))
        val = min(max(k * min_val - k * r + 1, 0.0), 1.0)
        return val

    def eval_move_dir(self):
        R = max(self.ell_a, self.ell_b)
        d = strech_vector_to(self.direction, R)
        avg_pos_diff = np.sum(self.pts, axis=0)
        com_pos_diff = self.orig_com - self.com

        apd_vec = strech_vector_to(d, vector_projection(avg_pos_diff, d))
        cpd_vec = strech_vector_to(d, vector_projection(com_pos_diff, d))

        apd_sign = -1 if xor(apd_vec[0] > 0, d[0] > 0) else 1
        cpd_sign = -1 if xor(cpd_vec[0] > 0, d[0] > 0) else 1

        ap_part = 1 - (R + (apd_sign * np.linalg.norm(apd_vec) / (2 * R)))
        cp_part = 1 - (R + (cpd_sign * np.linalg.norm(cpd_vec) / (2 * R)))

        return ap_part, cp_part

    def eval_support_shape(self):
        max_ps_coaf = 16

        cw_pts = [self.pts[cw_seq[self.pidx+i]]
                  for i in range(-len(cw_seq), 0)]
        perimeter = np.sum(
            np.array([np.linalg.norm(cw_pts[i] - cw_pts[i-1]) for i in range(4)]))
        pair1 = (area(cw_pts[0], cw_pts[1], cw_pts[2]),
                 area(cw_pts[2], cw_pts[3], cw_pts[0]))
        pair2 = (area(cw_pts[1], cw_pts[2], cw_pts[3]),
                 area(cw_pts[3], cw_pts[0], cw_pts[1]))
        surface = (pair1[0] + pair1[1] + pair2[0] + pair2[1]) / 2

        val = 1 - ((max_ps_coaf * surface) / (perimeter ** 2))
        return val

    def eval(self):
        prox_c = p.readUserDebugParameter(ev_prox_par)
        std_c = p.readUserDebugParameter(ev_std_par)
        avg_c = p.readUserDebugParameter(ev_avg_par)
        edge_c = p.readUserDebugParameter(ev_edge_par)
        ap_dir_c = p.readUserDebugParameter(ev_ap_dir_par)
        cp_dir_c = p.readUserDebugParameter(ev_cp_dir_par)
        sup_tri_c = p.readUserDebugParameter(ev_sup_tri_par)

        # print(f"coafs:{[prox_c, std_c, avg_c, ap_dir_c, cp_dir_c, sup_tri_c]}")

        val = 0
        prox_part = self.eval_min_prox()
        val += prox_c * prox_part
        avg_part, std_part, edge_part = self.eval_sequence()
        val += avg_c * avg_part
        val += std_c * std_part
        val += edge_c * edge_part
        apd_part, cpd_part = self.eval_move_dir()
        val += ap_dir_c * apd_part
        val += cp_dir_c * cpd_part
        sup_shp_part = self.eval_support_shape()
        val += sup_tri_c * sup_shp_part

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


def area(p1, p2, p3):
    return abs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])) / 2.0)


def init_debug_params():
    global ev_prox_par, ev_avg_par, ev_std_par, ev_edge_par, ev_ap_dir_par, ev_cp_dir_par, ev_sup_tri_par

    ev_prox_par = p.addUserDebugParameter("prox", 0.0, 1., EV_MIN_PROX_C)
    ev_avg_par = p.addUserDebugParameter("avg", 0.0, 1., EV_AVG_C)
    ev_std_par = p.addUserDebugParameter("std", 0.0, 1., EV_STD_C)
    ev_edge_par = p.addUserDebugParameter("edge", 0.0, 1., EV_EDGE_C)
    ev_ap_dir_par = p.addUserDebugParameter("ap_dir", 0.0, 1., EV_AP_DIR_C)
    ev_cp_dir_par = p.addUserDebugParameter("cp_dir", 0.0, 1., EV_CP_DIR_C)
    ev_sup_tri_par = p.addUserDebugParameter(
        "sup_tri_dir", 0.0, 1., EV_SUP_TRI_C)
