import math
import numpy as np
from enum import Enum
from numba import njit

MAX_DIP = -0.35
MIN_DIP = -0.1
LEG_TAR_H = -0.26
T_RAD = 0.012
MAX_WALK_H = -0.13
MIN_WALK_H = -0.20
SOFT_HIT_THR = 0.02

cw_seq = [0, 1, 3, 2]

############
#  UNUSED  #
############

WALK_H = -0.13
MIN_PERIOD = 0.02
ADJUST_SENS = abs(T_RAD) * 0.05
STEP = 0.01
F_UNSET = float("NaN")
LEG_ELLIPSE_A = 0.21
LEG_ELLIPSE_B = 0.18

# Grad. decend evaluation constants
EV_MIN_PROX_C = 0.1
EV_AVG_C = 0.35
EV_STD_C = 0.15
EV_EDGE_C = 0.5
EV_AP_DIR_C = 0.3
EV_CP_DIR_C = 0.05
EV_SUP_TRI_C = 0.2
EV_JUNCT_C = 0.2
EV_COM_BAL_C = 0.3


def f_arr_unset():
    return np.array([F_UNSET, F_UNSET, F_UNSET])

#######################
#  CHANGEBLE CONFIGS  #
#######################


LEG_EXC = 1.0  # to be changed in main.py


def change_leg_exc(val):
    global LEG_EXC
    LEG_EXC = val


@njit
def get_leg_exc():
    return LEG_EXC
