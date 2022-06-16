import math
import numpy as np
from enum import Enum


MAX_HEIGHT = 1.0
MAX_DIP = -0.2
MAX_FREQ = 50
MIN_PERIOD = 0.02
MAX_DROP_SPEED = -0.2
T_RAD = 0.05
ADJUST_SENS = abs(T_RAD) * 0.05



# some aggregations
FRONT_DIR_LEFT = [1, -1, -1]
FRONT_DIR_RIGHT = [1, -1, 1]
BACK_DIR_LEFT = [-1, 1, -1]
BACK_DIR_RIGHT = [-1, 1, 1]
DIRECTIONS = [FRONT_DIR_LEFT, FRONT_DIR_RIGHT, BACK_DIR_LEFT, BACK_DIR_RIGHT]

# registering maximum angles for joints
restrictions = np.array([
    [-0.1, -0.1, -0.04, -0.15, -0.5, -0.8, -math.pi / 6, -math.pi / 9, -1, math.pi / 60],
    [0.1, 0.1, 0.04, 0.3, 0.5, 0.8, math.pi / 6, math.pi / 9, 1, math.pi / 6]]
)

# finite-state machine(FSM)
# [0-3) states: legs are on the ground(lower number means leg is further forward)
# 3 state: leg is lifted and is traveling forwards
#   mv1   mv2  mv3  mv4  st   it5  it6  it7  it8
step_table = np.array([
    [0, 1, 2, 3, 1, 2, 0.5, 1.5, 2.5],  # current state
    [2, 3, 0, 1, 1, 2, 2.5, 1.5, 0.5],
    [1, 2, 3, 0, 2, 1, 1.5, 2.5, 1.5],
    [3, 0, 1, 2, 2, 1, 1.5, 0.5, 1.5],

    #   mv2   mv3  mv4  mv1  it7  it8  mv2  mv3  mv4
    [1, 2, 3, 0, 1.5, 2.5, 1, 2, 3],  # next state if moving
    [3, 0, 1, 2, 1.5, 0.5, 3, 0, 1],
    [2, 3, 0, 1, 2.5, 1.5, 2, 3, 0],
    [0, 1, 2, 3, 0.5, 1.5, 0, 1, 2],

    #   it6   mv3  it8  mv1  st   it6  st   it5  st
    [0.5, 2, 2.5, 0, 1, 0.5, 1, 2, 1],  # next state if stopping
    [2.5, 0, 0.5, 2, 1, 2.5, 1, 2, 1],
    [1.5, 3, 1.5, 1, 2, 1.5, 2, 1, 2],
    [1.5, 1, 1.5, 3, 2, 1.5, 2, 1, 2],

    [-1, 0, 1, 0, -1, -1, -1, 0, 1],  # data for drift_coef
    [-1, 0, 1, 0, 0, 1, 0, 0, 0]
])


# some constants and global variables
state_table = np.array([[1.0, 1.0, 2.0, 2.0], [1.0, 1.0, 2.0, 2.0]])
after_air_tick_count = 5
air_tick_count = 11
base_force_vector = np.array([0, 0, 0], dtype=np.float)
base_frame_orientation_matrix = np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]])

base_orientation_matrix = np.zeros((3, 3))
base_shock_threshold = 2.5
clock_wise_sequence = [0, 1, 3, 2]
drift_coaf = 0  # shows how much bot leans to side at certain time
drift_table = np.array([0.0, 0.0])  # used with drift_coaf
go_threshold = 0.001
grounded_legs = np.array([False, False, False, False])
horizontal_turn_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
last_movement_properties = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
last_zs = np.array([0.0, 0.0, 0.0, 0.0])
legs_height_pos = np.array([0.0, 0.0, 0.0, 0.0])
legs_action_pos = np.array([0, 0, 0, 0])
rebound_container = np.zeros(5)
shock_threshold = 37.8 * 1
starting_positions = np.array(
    [[0.125, -0.065, -0.24], [0.125, 0.065, -0.24], [-0.125, -0.065, -0.24], [-0.125, 0.065, -0.24]])
movement_command = 0
positions = np.array([[0.0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
action_partial = 0.0
process_position = 0.0
process_state = 0
touch_force = [0, 0, 0, 0]
touch_force_max = [0, 0, 0, 0]