import pybullet as p
import time
import numpy as np
import math
import threading

from consts import *
from leg import Leg
from quad import Quad


def to_starting_position():
    global positions
    positions = starting_positions.copy()
    p.setJointMotorControl2(q.model, q.front_left_leg.base, p.POSITION_CONTROL, -0.08354599985540861)
    p.setJointMotorControl2(q.model, q.front_left_leg.shoulder, p.POSITION_CONTROL, -0.7116395204323857)
    p.setJointMotorControl2(q.model, q.front_left_leg.knee, p.POSITION_CONTROL, 1.423279103830849)
    p.setJointMotorControl2(q.model, q.front_left_leg.heel, p.POSITION_CONTROL, -1.423279103830849)
    p.setJointMotorControl2(q.model, q.back_left_leg.base, p.POSITION_CONTROL, -0.08354599985540861)
    p.setJointMotorControl2(q.model, q.back_left_leg.shoulder, p.POSITION_CONTROL, 0.7116395204323857)
    p.setJointMotorControl2(q.model, q.back_left_leg.knee, p.POSITION_CONTROL, -1.423279103830849)
    p.setJointMotorControl2(q.model, q.back_left_leg.heel, p.POSITION_CONTROL, 1.423279103830849)
    p.setJointMotorControl2(q.model, q.front_right_leg.base, p.POSITION_CONTROL, 0.08354599985540861)
    p.setJointMotorControl2(q.model, q.front_right_leg.shoulder, p.POSITION_CONTROL, -0.7116395204323857)
    p.setJointMotorControl2(q.model, q.front_right_leg.knee, p.POSITION_CONTROL, 1.423279103830849)
    p.setJointMotorControl2(q.model, q.front_right_leg.heel, p.POSITION_CONTROL, -1.423279103830849)
    p.setJointMotorControl2(q.model, q.back_right_leg.base, p.POSITION_CONTROL, 0.08354599985540861)
    p.setJointMotorControl2(q.model, q.back_right_leg.shoulder, p.POSITION_CONTROL, 0.7116395204323857)
    p.setJointMotorControl2(q.model, q.back_right_leg.knee, p.POSITION_CONTROL, -1.423279103830849)
    p.setJointMotorControl2(q.model, q.back_right_leg.heel, p.POSITION_CONTROL, 1.423279103830849)


# gets info from sensors
def update_sensor_info():
    global base_orientation
    global base_force_vector
    global touch_force
    global base_orientation_matrix
    global base_frame_orientation_matrix
    global base_position
    base_position = np.array(p.getBasePositionAndOrientation(q.model)[0])
    base_force_vector = -np.array(p.getJointState(q.model, q.sensor)[2][slice(3)])
    base_orientation = np.array(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(q.model)[1]))
    base_orientation_1d = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(q.model)[1]))
    for i in range(9):
        base_orientation_matrix[i // 3][i % 3] = base_orientation_1d[i]
    base_frame_orientation_matrix = get_base_frame_orientation_matrix()
    touch_force = touch_force_max


def get_commands():
    global last_movement_properties
    global touch_force_max
    global last_time
    global starting_speed
    global movement_command

    movement_command = p.readUserDebugParameter(param2)
    update_sensor_info()

    # reads debugging parameters as commands
    last_movement_properties = np.array([
        [p.readUserDebugParameter(step),
         p.readUserDebugParameter(cross_step),
         p.readUserDebugParameter(sit_up),
         p.readUserDebugParameter(speed)],
        [p.readUserDebugParameter(side_lean),
         p.readUserDebugParameter(inclination),
         p.readUserDebugParameter(turn),
         p.readUserDebugParameter(front_lean)],
        [p.readUserDebugParameter(step_height),
         p.readUserDebugParameter(spread), 0, 0]])

    # make_movement_args()
    # update_legs()
    # for l in q.legs:
    #     l.update(q)
    # ang = np.array([p.readUserDebugParameter(alpha_par),
    #      p.readUserDebugParameter(beta_par),
    #      p.readUserDebugParameter(gama_par)])
    cor = np.array([p.readUserDebugParameter(x_par),
         p.readUserDebugParameter(y_par),
         p.readUserDebugParameter(z_par)])

    for l in q.legs:
        l.update(q, cor)

    # q.front_left_leg.update(q, cor)
    # q.back_left_leg.update(q, cor)
    # q.front_right_leg.update(q, cor)
    # q.back_right_leg.update(q, cor)

    touch_force_max = np.array([0, 0, 0, 0])


# updates angles for each joint
# def update_legs():
#     for i in range(4):
#         angles = get_angles(LEGS[i], positions[i], DIRECTIONS[i])
#
#         p.setJointMotorControl2(q.model, LEGS[i][0], p.POSITION_CONTROL, angles[2],
#                                 force=p.getJointInfo(q.model, LEGS[i][0])[10] / 100,
#                                 maxVelocity=p.getJointInfo(q.model, LEGS[i][0])[11])
#         p.setJointMotorControl2(q.model, LEGS[i][1], p.POSITION_CONTROL, angles[1],
#                                 force=p.getJointInfo(q.model, LEGS[i][1])[10] / 200,
#                                 maxVelocity=p.getJointInfo(q.model, LEGS[i][1])[11] * 2)
#         p.setJointMotorControl2(q.model, LEGS[i][2], p.POSITION_CONTROL, angles[0],
#                                 force=p.getJointInfo(q.model, LEGS[i][2])[10] / 100,
#                                 maxVelocity=p.getJointInfo(q.model, LEGS[i][2])[11])


# this does conversion from cartesian to polar coordinate system
def get_angles(leg, target_position, dir):
    r = 0.1
    target = -np.array([-1 * p.getJointInfo(q.model, leg[0])[14][0], p.getJointInfo(q.model, leg[0])[14][1],
                        p.getJointInfo(q.model, leg[0])[14][2]]) + np.array(target_position)
    gama = math.atan(-target[1] / target[2]) - math.atan(
        p.getJointInfo(q.model, leg[1])[14][1] / np.linalg.norm(target))
    if p.getJointInfo(q.model, leg[0])[8] > gama:
        gama = p.getJointInfo(q.model, leg[0])[8]
        target[1] = target[2] * math.tan(gama)
    elif p.getJointInfo(q.model, leg[0])[9] < gama:
        gama = p.getJointInfo(q.model, leg[0])[9]
        target[1] = target[2] * math.tan(gama)
    shoulder_position = np.array([dir[1] * 0.025, (0.03 * math.sin(gama) + dir[2] * 0.01 * math.cos(gama)),
                                  (-0.03 * math.cos(gama) + dir[2] * 0.01 * math.sin(gama))])
    target = target - shoulder_position
    leg_length = np.linalg.norm(target)
    fi = math.asin(target[0] / leg_length)

    if 0.1931851653 > leg_length > r:  # middle
        e = math.acos((-(leg_length ** 2)) / (2 * r * r) + 1)
        alpha = dir[0] * (math.pi - e)
        if p.getJointInfo(q.model, leg[1])[8] + alpha / 2 <= fi <= p.getJointInfo(q.model, leg[1])[9] + alpha / 2:
            beta = fi - alpha / 2  # VII
        elif fi < p.getJointInfo(q.model, leg[1])[8] + alpha / 2:
            beta = p.getJointInfo(q.model, leg[1])[8]  # VIII
        else:
            beta = p.getJointInfo(q.model, leg[1])[9]
            alpha += 1 * ((fi - alpha / 2) - beta)  # IX
    elif leg_length >= 0.1931851653:  # outer
        alpha = p.getJointInfo(q.model, leg[2])[8]
        if p.getJointInfo(q.model, leg[1])[8] + dir[0] * 0.2617993878 <= fi <= \
                p.getJointInfo(q.model, leg[1])[9] + dir[0] * 0.2617993878:
            beta = fi - dir[0] * 0.2617993878  # I
        elif fi < p.getJointInfo(q.model, leg[1])[8] + dir[0] * 0.2617993878:
            beta = p.getJointInfo(q.model, leg[1])[8]  # II
        else:
            beta = p.getJointInfo(q.model, leg[1])[9]  # III
    else:  # inner
        alpha = p.getJointInfo(q.model, leg[2])[9]
        if p.getJointInfo(q.model, leg[1])[8] + dir[0] * 1.047197552 <= fi <= \
                p.getJointInfo(q.model, leg[1])[9] + dir[0] * 1.047197552:
            beta = fi - dir[0] * 1.047197552  # IV
        elif fi <= p.getJointInfo(q.model, leg[1])[8] + dir[0] * 1.047197552:
            beta = p.getJointInfo(q.model, leg[1])[8]  # V
        else:
            beta = p.getJointInfo(q.model, leg[1])[9]  # VI
    return [alpha, beta, gama]


# bot walking function
def walk(movement, orientation,
         other):  # [forward_step, side_step, sit_up_height, speed], [side_lean, inclination, turn, fron_lean], [step_height, spread]
    global legs_action_pos
    global last_zs
    global grounded_legs
    global positions

    set_movement_state(last_movement_properties[0][3])  # change state if needed
    movement[3] = abs(movement[3])  # make speed absolute
    change_by_velocity = 1.3 - (
                3380 * (movement[3] + 0.0135) / 627) ** 0.5  # leaning side to side for additional stability
    for i in range(4):  # for each leg
        if legs_height_pos[i] != 0:  # leg is lifted
            grounded_legs[i] = False
            z = 0.1217395 * (
                    0.410713 * math.sqrt(1 - (2 * legs_height_pos[i] - 1) ** 2) + 2 * other[0] * legs_height_pos[
                i])  # construct height(based on ellipse function)
        else:
            grounded_legs[i] = True  # basically the same as legs_height_pos[i] == 0:
            z = 0
        if movement[0] > 0.07 and ((i < 2 and legs_action_pos[i] < 0) or (i >= 2 and legs_action_pos[i] >= 0)):
            R = 0.07
        else:
            R = movement[0]
        x = R * 1.217395 * legs_action_pos[
            i]  # construct x advancement(based on ellipse function, but constants were simplified)
        y = movement[1] * legs_action_pos[i] * 0.75  # construct y for side step
        if i % 2 == 0:  # left leg: -1; right leg: 1
            spread_dir = -1
        else:
            spread_dir = 1
        z += spread_dir * change_by_velocity * 0.007 * math.sin(
            (math.pi * drift_coaf) / 2)  # leaning side to side for additional stability
        y += change_by_velocity * -0.02 * math.sin(
            (math.pi * drift_coaf) / 2)  # leaning side to side for additional stability

        pos = starting_positions[i] + np.array([x, y, z])  # gets finger position for each leg
        # switching to polar coordinate system in OxOy plane (ro, fi)
        ro = math.sqrt(pos[0] ** 2 + pos[1] ** 2)  # distance to finger from center in OxOy plane
        fi = math.atan(pos[1] / pos[0])  # angle form Oy axis

        front_lean_height = pos[0] * math.tan(orientation[3])
        if i > 1:  # reversed for rear legs
            ro = -ro
        pos[0] = ro * math.cos(fi + legs_action_pos[i] * orientation[2]) - 0.15 * math.tan(
            orientation[1])  # X axis position associated with turn and inclination
        pos[1] = ro * math.sin(fi + legs_action_pos[i] * orientation[2]) - spread_dir * pos[2] * math.sin(
            other[1])  # Y axis position associated with turn and spread
        pos[2] -= (movement[2] - pos[1] * math.tan(orientation[0])) + (
                    pos[2] - pos[2] * math.cos(other[1])) + front_lean_height  # we use cartesian so we can use -=.
        positions[i] = pos  # saving new positions
        last_zs[i] = z  # recording last height


# determines if bot state has to change and changes it
def set_movement_state(sp):
    global legs_action_pos
    global legs_height_pos
    global action_partial
    global state_table
    global drift_coaf
    if action_partial + sp >= 1:  # action advances enough to change state
        action_partial = (action_partial + sp) % 1
        if round(sp, 4) > 0.01 and round(movement_command,
                                         ndigits=3) >= go_threshold:  # starting speed is used as start/stop walking parameter, need to fix names todo
            action = 1  # go
        else:
            action = 2  # stop
        next_state = get_state_FSM(state_table[1], action)
        state_table[0] = state_table[1]
        state_table[1] = next_state.flatten()
    else:  # state stays the same
        action_partial = action_partial + sp
    state_uncomp = state_table[0] + (state_table[1] - state_table[0]) * action_partial
    legs_action_pos = 2 * (1.5 - state_uncomp) / 3
    # print(legs_action_pos)
    drift_coaf = drift_table[0] + (drift_table[1] - drift_table[0]) * action_partial
    for i in range(4):
        if state_table[1][i] - state_table[0][i] < 0:
            legs_height_pos[i] = action_partial  # leg is lifted
        else:
            legs_height_pos[i] = 0.0  # len is on the ground


# gets new state from finite-state machine
def get_state_FSM(state, action):
    global drift_table
    for i in range(step_table.shape[1]):
        if np.array_equal(state, step_table[:4, i]):  # searching for current state in FSM
            drift_table[0] = drift_table[1]  # extracting additional data
            drift_table[1] = step_table[11 + action, i]
            return step_table[action * 4:(action + 1) * 4, i]  # returning next state
    return np.array([[0, 0, 0, 0], [0, 0, 0, 0]])


# some math, nothing to see here
def get_base_frame_orientation_matrix():
    global horizontal_turn_matrix
    for i in range(4):
        if grounded_legs[clock_wise_sequence[i]] and grounded_legs[clock_wise_sequence[i - 1]] and \
                grounded_legs[clock_wise_sequence[i - 2]]:
            forward = np.array([-1, 0, 0])
            X = base_orientation_matrix.dot(forward)
            if X[0] <= 0:
                temp = math.atan(X[1] / X[0])
            else:
                temp = math.atan(X[1] / X[0]) - math.pi
            h = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, 0, -temp]))
            horizontal_turn_m = np.zeros((3, 3))
            for j in range(9):
                horizontal_turn_m[j // 3][j % 3] = h[j]
            horizontal_turn_matrix = horizontal_turn_m.copy()
            new_matrix = np.matmul(horizontal_turn_m, base_orientation_matrix)
            return new_matrix
    return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])


def shock_control():
    map = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]])
    av_map = np.array([0, 0])
    av_count = 0
    max_coordinate = 0
    force_container = 0
    shock = False
    mid_air = True
    for i in range(4):
        if touch_force_max[clock_wise_sequence[i]] != 0:
            mid_air = False
        if touch_force_max[clock_wise_sequence[i]] > shock_threshold:
            av_map = av_map + map[i] * (touch_force_max[clock_wise_sequence[i]] - shock_threshold)
            force_container += (touch_force_max[clock_wise_sequence[i]] - shock_threshold)
            av_count += 1
            shock = True
            if abs((map[i] * (touch_force_max[clock_wise_sequence[i]] - shock_threshold))
                   [0]) > abs((map[i] * (touch_force_max[clock_wise_sequence[i]] - shock_threshold))[1]):
                bigger = (map[i] * (touch_force_max[clock_wise_sequence[i]] - shock_threshold))[0]
            else:
                bigger = (map[i] * (touch_force_max[clock_wise_sequence[i]] - shock_threshold))[1]
            if abs(bigger) > max_coordinate:
                max_coordinate = abs(bigger)
    if shock and abs(base_force_vector[2]) >= base_shock_threshold:
        av_map = av_map / (av_count * max_coordinate)
        return [1, np.array([-1, 1]) * np.array([av_map[0], av_map[1]]), force_container]
    elif not mid_air:
        return [0, np.array([0, 0]), force_container]
    else:
        return [-1, np.array([0, 0]), force_container]


def shock_damping():
    global air_tick_count
    global after_air_tick_count
    global last_movement_properties
    global rebound_container

    f_step = last_movement_properties[0][0]
    fl = last_movement_properties[1][3]
    sl = last_movement_properties[1][0]
    incl = last_movement_properties[1][1]
    h = last_movement_properties[0][2]
    spread = last_movement_properties[2][1]
    speed = last_movement_properties[0][3]
    turn = last_movement_properties[1][2]
    step_height = last_movement_properties[2][0]
    side_step = last_movement_properties[0][1]

    s = shock_control()

    if after_air_tick_count >= 0:
        after_air_tick_count -= 1
    if s[0] == -1:
        air_tick_count += 1
    else:
        air_tick_count = 0
    if air_tick_count >= 3:
        stand_dir = np.array([-1, -1, 1]) * base_frame_orientation_matrix.dot(np.array([0, 0, 1]))
        # speed = 0
        turn = 0
        step_height = 0
        side_step = 0
        after_air_tick_count = 5
        diff = ((-stand_dir[0] / stand_dir[2]) - incl) * 0.4
        rebound_container[3] = check_restrictions_reb(rebound_container[3], incl, diff, 5)
        incl += diff
        diff = (abs(stand_dir[1] / stand_dir[2]) - spread) * 0.2
        rebound_container[4] = check_restrictions_reb(rebound_container[4], spread, diff, 9)
        spread += diff

        # print("in air")
        if last_movement_properties[0][2] + 0.01 < 0.02:
            diff = 0.01
            rebound_container[0] = check_restrictions_reb(rebound_container[0], h, diff, 2)
            h += diff
        else:
            diff = 0.02 - last_movement_properties[0][2]
            rebound_container[0] = check_restrictions_reb(rebound_container[0], h, diff, 2)
            h += diff
        if abs(last_movement_properties[1][3]) != 0:
            diff = -0.1 * math.atan(last_movement_properties[1][3] * 10)
            rebound_container[1] = check_restrictions_reb(rebound_container[1], fl, diff, 7)
            fl += diff
        if abs(last_movement_properties[1][0]) != 0:
            diff = -0.05 * math.atan(last_movement_properties[1][0] * 20)
            rebound_container[2] = check_restrictions_reb(rebound_container[2], sl, diff, 4)
            sl += diff
    elif s[0] == 1:
        f_force = s[2] * s[1][0]
        s_force = s[2] * s[1][1]
        stand_dir = np.array([-1, -1, 1]) * base_frame_orientation_matrix.dot(np.array([0, 0, -1]))

        diff = ((-stand_dir[0] / stand_dir[2]) - incl) * 0.4
        rebound_container[3] = check_restrictions_reb(rebound_container[3], incl, diff, 5)
        incl += diff
        diff = (abs(stand_dir[1] / stand_dir[2]) - spread) * 0.2
        rebound_container[4] = check_restrictions_reb(rebound_container[4], spread, diff, 9)
        spread += diff

        diff = 0.1 * f_force / shock_threshold
        rebound_container[1] = check_restrictions_reb(rebound_container[1], fl, diff, 7)
        fl += diff

        diff = 0.1 * s_force / shock_threshold
        rebound_container[2] = check_restrictions_reb(rebound_container[2], sl, diff, 4)
        sl += diff

        diff = 0.05 * ((base_force_vector[2] / base_shock_threshold) + 1)
        rebound_container[0] = check_restrictions_reb(rebound_container[0], h, diff, 2)
        h += diff
        print("shock", touch_force)
    else:
        rebounding_consts = np.array([0.2, 0.2, 0.25, 0.2, 0.2])  # h fl sl incl spread
        decr = rebounding_consts[0] * rebound_container[0]
        rebound_container[0] -= decr
        h -= decr

        decr = rebounding_consts[1] * rebound_container[1]
        rebound_container[1] -= decr
        fl -= decr

        decr = rebounding_consts[2] * rebound_container[2]
        rebound_container[2] -= decr
        sl -= decr

        decr = rebounding_consts[3] * rebound_container[3]
        rebound_container[3] -= decr
        incl -= decr

        decr = rebounding_consts[4] * rebound_container[4]
        rebound_container[4] -= decr
        spread -= decr

    last_movement_properties = np.array(
        [[f_step, side_step, h, speed], [sl, incl, turn, fl], [step_height, spread, 0, 0]])


def make_movement_args():
    global last_movement_properties

    shock_damping()

    check_restrictions_universal(last_movement_properties)
    walk(last_movement_properties[0], last_movement_properties[1], last_movement_properties[2])


# checks if certain joint fits in its predefined limits
def check_restrictions_universal(prop, indices=tuple(range(10))):
    i = 0
    try:
        for pr in np.nditer(prop, op_flags=['readwrite']):
            if pr > restrictions[1, indices[i]]:
                pr[...] = restrictions[1, indices[i]]
            else:
                if pr < restrictions[0, indices[i]]:
                    pr[...] = restrictions[0, indices[i]]
            i += 1
        return prop
    except IndexError:
        return


def check_restrictions_reb(prop, current_value, addition, index):
    if current_value + addition > restrictions[1, index]:
        addition = restrictions[1, index] - current_value
        if addition > 0:
            prop += addition
    elif current_value + addition < restrictions[0, index]:
        addition = restrictions[0, index] - current_value
        if addition < 0:
            prop += addition
    else:
        prop = max(restrictions[0, index], min(restrictions[1, index], prop + addition))
    return prop


# # updates pressure sensors
# def high_freq():
#     p.stepSimulation()
#     for i in range(4):
#         touch_force_max[i] = max(-p.getJointState(q.model, q.sensors[i])[2][2], touch_force_max[i])


# start point

p.connect(p.GUI)
plane = p.createCollisionShape(p.GEOM_PLANE)
# ball = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)

# p.createMultiBody(0, ball, basePosition= [1., 0., 0.4])
p.createMultiBody(0, plane)
p.setGravity(0, 0, -10)
useMaximalCoordinates = False

model = p.loadURDF("quad2.xacro", [0.0, 0.0, 0.2], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
# model = p.loadURDF("quad2.xacro", [0.0, 0.0, 0.2], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)

# Getting joints indices
nJoints = p.getNumJoints(model)
jointNameToId = {}
for k in range(nJoints):
    jointInfo = p.getJointInfo(model, k)
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

base_j = jointNameToId['front_left_base_to_shoulder']
shoulder_j = jointNameToId['front_left_shoulder']
knee_j = jointNameToId['front_left_knee']
heel_j = jointNameToId['front_left_heel']
sensor_j = jointNameToId['front_left_sensor']
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
fll = Leg("front_left", base_j, shoulder_j, knee_j, heel_j, sensor_j, FRONT_DIR_LEFT, [0.125, -0.065, -0.24], off)

base_j = jointNameToId['back_left_base_to_shoulder']
shoulder_j = jointNameToId['back_left_shoulder']
knee_j = jointNameToId['back_left_knee']
heel_j = jointNameToId['back_left_heel']
sensor_j = jointNameToId['back_left_sensor']
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
bll = Leg("back_left", base_j, shoulder_j, knee_j, heel_j, sensor_j, BACK_DIR_LEFT, [0.125, 0.065, -0.24], off)

base_j = jointNameToId['front_right_base_to_shoulder']
shoulder_j = jointNameToId['front_right_shoulder']
knee_j = jointNameToId['front_right_knee']
heel_j = jointNameToId['front_right_heel']
sensor_j = jointNameToId['front_right_sensor']
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
frl = Leg("front_right", base_j, shoulder_j, knee_j, heel_j, sensor_j, FRONT_DIR_RIGHT, [-0.125, -0.065, -0.24], off)

base_j = jointNameToId['back_right_base_to_shoulder']
shoulder_j = jointNameToId['back_right_shoulder']
knee_j = jointNameToId['back_right_knee']
heel_j = jointNameToId['back_right_heel']
sensor_j = jointNameToId['back_right_sensor']
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
brl = Leg("back_right", base_j, shoulder_j, knee_j, heel_j, sensor_j, BACK_DIR_RIGHT, [-0.125, 0.065, -0.24], off)

sensor_b = jointNameToId['base_sensor']  # we can use this as accelerometer and gyroscope
q: Quad = Quad(model, fll, frl, bll, brl, sensor_b)


base_orientation = np.array(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(q.model)[1]))
base_position = np.array(p.getBasePositionAndOrientation(q.model)[0])
touch_force_max = np.array([0, 0, 0, 0])
# starting simulation
# p.setRealTimeSimulation(0)
p.setTimeStep(timeStep=1/50)


# setting manual parameters for debugging
speed = p.addUserDebugParameter("speed", -0.15, 0.3, 0)
step = p.addUserDebugParameter("step", -0.1, 0.1, 0)
cross_step = p.addUserDebugParameter("cross step", -0.1, 0.1, 0)
turn = p.addUserDebugParameter("turn", -0.5235988, 0.5235988, 0)
side_lean = p.addUserDebugParameter("side lean", -0.5, 0.5, 0.0)
front_lean = p.addUserDebugParameter("front lean", -math.pi / 9, math.pi / 9, 0)
inclination = p.addUserDebugParameter("inclination", -0.8, 0.8, 0.0)
sit_up = p.addUserDebugParameter("sit up", -0.04, 0.04, 0)
step_height = p.addUserDebugParameter("step height", 0, 0.3, 0)
spread = p.addUserDebugParameter("spread", 0, math.pi / 6, math.pi / 60)

param2 = p.addUserDebugParameter("stop/go", 0, 0.001, 0.)

# alpha_par = p.addUserDebugParameter("alpha", -2.617993877991494, -0.52359877559829884, -1.5707)
# beta_par = p.addUserDebugParameter("beta", -math.pi/2, math.pi/2, 0)
# gama_par = p.addUserDebugParameter("gama", -math.pi/2, math.pi/2, 0)
x_par = p.addUserDebugParameter("x", -0.3, 0.3, 0)
y_par = p.addUserDebugParameter("y", -0.2, 0.2, -0.0)
z_par = p.addUserDebugParameter("z", -0.4, -0.1, -0.26)


# increase grip
for j in range(-1, 21):
    p.changeDynamics(q.model, j, lateralFriction=2)
    print(p.getDynamicsInfo(q.model, j))
for j in range(0, 21, 5):
    print(p.getJointInfo(q.model, j)[1])
    p.enableJointForceTorqueSensor(q.model, j, 1)
# p.enableJointForceTorqueSensor(q.model, q.front_left_leg.heel, 1)

# to_starting_position()  # sets robot to starting position
counter = 0

INTERV = 20
l_time = int(1000 * time.time())

while 1:
    c_time = int(1000 * time.time())
    if c_time >= l_time + INTERV:
        l_time = c_time
        for i in range(4):
            touch_force_max[i] = max(-p.getJointState(q.model, q.sensors[i])[2][2], touch_force_max[i])
        get_commands()
        p.stepSimulation()
    else:
        time.sleep(0.005)

    # high_freq()  # measuring pressure sensors more often
    # if counter % 5 == 0:
    #     t = threading.Thread(target=get_commands)  # base frequency is 50 Hz
    #     t.start()
    # counter += 1
    #
    # time.sleep(1 / 250)
