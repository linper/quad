import pybullet as p
# import matplotlib.pyplot as plt
# import pandas as pd
import time
import numpy as np
import math
import threading
import queue


def toStartingPosition():
    global positions
    positions = starting_positions.copy()
    p.setJointMotorControl2(quad, front_left_base_to_shoulder, p.POSITION_CONTROL, -0.08354599985540861)
    p.setJointMotorControl2(quad, front_left_shoulder, p.POSITION_CONTROL, -0.7116395204323857)
    p.setJointMotorControl2(quad, front_left_knee, p.POSITION_CONTROL, 1.423279103830849)
    p.setJointMotorControl2(quad, back_left_base_to_shoulder, p.POSITION_CONTROL, -0.08354599985540861)
    p.setJointMotorControl2(quad, back_left_shoulder, p.POSITION_CONTROL, 0.7116395204323857)
    p.setJointMotorControl2(quad, back_left_knee, p.POSITION_CONTROL, -1.423279103830849)
    p.setJointMotorControl2(quad, front_right_base_to_shoulder, p.POSITION_CONTROL, 0.08354599985540861)
    p.setJointMotorControl2(quad, front_right_shoulder, p.POSITION_CONTROL, -0.7116395204323857)
    p.setJointMotorControl2(quad, front_right_knee, p.POSITION_CONTROL, 1.423279103830849)
    p.setJointMotorControl2(quad, back_right_base_to_shoulder, p.POSITION_CONTROL, 0.08354599985540861)
    p.setJointMotorControl2(quad, back_right_shoulder, p.POSITION_CONTROL, 0.7116395204323857)
    p.setJointMotorControl2(quad, back_right_knee, p.POSITION_CONTROL, -1.423279103830849)


def updateSensorInfo():
    global base_orientation
    global base_force_vector
    global touch_force
    global base_orientation_matrix
    global base_frame_orientation_matrix
    global base_position
    global acceleration
    base_position = np.array(p.getBasePositionAndOrientation(quad)[0])
    base_force_vector = -np.array(p.getJointState(quad, base_sensor)[2][slice(3)])
    base_orientation = np.array(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(quad)[1]))
    base_orientation_1d = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(quad)[1]))
    for i in range(9):
        base_orientation_matrix[i // 3][i % 3] = base_orientation_1d[i]
    base_frame_orientation_matrix = getBaseFrameOrientationMatrix()
    acceleration = base_frame_orientation_matrix.dot(getAccelerationInfo(base_position))
    # for i in range(4):
    #     touch_force[i] = -p.getJointState(quad, SENSORS[i])[2][2]
    touch_force = touch_force_max


def getCommands():
    # global test_stop
    global last_movement_properties
    global touch_force_max
    global last_time
    global starting_speed
    # forced = False
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if k == ord('l') and (v & p.KEY_IS_DOWN):
            pass
            # toStartingPosition()
            # t = threading.Timer(0.02, getCommands)
            # t.start()
    # t = threading.Timer(p.readUserDebugParameter(param), getCommands)
    # t = threading.Timer(0.015, getCommands)
    starting_speed = p.readUserDebugParameter(param2)
    # t.start()
    updateSensorInfo()

    # walking([p.readUserDebugParameter(step),
    #              p.readUserDebugParameter(cross_step),
    #              p.readUserDebugParameter(sit_up),
    #              # 1 / round(1 / p.readUserDebugParameter(speed), 0)],
    #              p.readUserDebugParameter(speed)],
    #             [p.readUserDebugParameter(side_lean),
    #              p.readUserDebugParameter(inclination),
    #              p.readUserDebugParameter(turn),
    #              p.readUserDebugParameter(front_lean)],
    #             [p.readUserDebugParameter(step_height),
    #              p.readUserDebugParameter(spread)])

    # setMovementState(p.readUserDebugParameter(speed), p.readUserDebugParameter((turn)))
    # walking2([p.readUserDebugParameter(step),
    #              p.readUserDebugParameter(cross_step),
    #              p.readUserDebugParameter(sit_up),
    #              # 1 / round(1 / p.readUserDebugParameter(speed), 0)],
    #              p.readUserDebugParameter(speed)],
    #             [p.readUserDebugParameter(side_lean),
    #              p.readUserDebugParameter(inclination),
    #              p.readUserDebugParameter(turn),
    #              p.readUserDebugParameter(front_lean)],
    #             [p.readUserDebugParameter(step_height),
    #              p.readUserDebugParameter(spread)])

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
    # checkRestrictionsUniversal(last_movement_properties)
    # setMovementState(last_movement_properties[0][3], last_movement_properties[1][2])
    # walking2(last_movement_properties[0], last_movement_properties[1], last_movement_properties[2])

    # makeMovementArgs(p.readUserDebugParameter(speed), p.readUserDebugParameter(turn),
    #                  p.readUserDebugParameter(cross_step), p.readUserDebugParameter(step_height))
    makeMovementArgsLite()
    # if not test_stop:#TODO
    updateLegs()
    touch_force_max = np.array([0, 0, 0, 0])
    # if forced:
    #     test_stop = True
    #     forced = False


def updateLegs():
    for i in range(4):
        angles = getAngles(LEGS[i], positions[i], DIRECTIONS[i])
        # p.setJointMotorControl2(quad, LEGS[i][0], p.POSITION_CONTROL, angles[2],
        #     force=p.getJointInfo(quad, LEGS[i][0])[10], maxVelocity=p.getJointInfo(quad, LEGS[i][0])[11])
        # p.setJointMotorControl2(quad, LEGS[i][1], p.POSITION_CONTROL, angles[1],
        #     force=p.getJointInfo(quad, LEGS[i][1])[10], maxVelocity=p.getJointInfo(quad, LEGS[i][1])[11])
        # p.setJointMotorControl2(quad, LEGS[i][2], p.POSITION_CONTROL, angles[0],
        #     force=p.getJointInfo(quad, LEGS[i][2])[10], maxVelocity=p.getJointInfo(quad, LEGS[i][2])[11])

        # p.setJointMotorControl2(quad, LEGS[i][0], p.POSITION_CONTROL, angles[2],
        #                         force=p.getJointInfo(quad, LEGS[i][0])[10] / 1,
        #                         maxVelocity=p.getJointInfo(quad, LEGS[i][0])[11])
        # p.setJointMotorControl2(quad, LEGS[i][1], p.POSITION_CONTROL, angles[1],
        #                         force=p.getJointInfo(quad, LEGS[i][1])[10] / 2,
        #                         maxVelocity=p.getJointInfo(quad, LEGS[i][1])[11] * 2)
        # p.setJointMotorControl2(quad, LEGS[i][2], p.POSITION_CONTROL, angles[0],
        #                         force=p.getJointInfo(quad, LEGS[i][2])[10] / 1,
        #                         maxVelocity=p.getJointInfo(quad, LEGS[i][2])[11])

        p.setJointMotorControl2(quad, LEGS[i][0], p.POSITION_CONTROL, angles[2],
                                force=p.getJointInfo(quad, LEGS[i][0])[10] / 100,
                                maxVelocity=p.getJointInfo(quad, LEGS[i][0])[11])
        p.setJointMotorControl2(quad, LEGS[i][1], p.POSITION_CONTROL, angles[1],
                                force=p.getJointInfo(quad, LEGS[i][1])[10] / 200,
                                maxVelocity=p.getJointInfo(quad, LEGS[i][1])[11] * 2)
        p.setJointMotorControl2(quad, LEGS[i][2], p.POSITION_CONTROL, angles[0],
                                force=p.getJointInfo(quad, LEGS[i][2])[10] / 100,
                                maxVelocity=p.getJointInfo(quad, LEGS[i][2])[11])

        # p.setJointMotorControl2(quad, LEGS[i][0], p.POSITION_CONTROL, angles[2])
        # p.setJointMotorControl2(quad, LEGS[i][1], p.POSITION_CONTROL, angles[1])
        # p.setJointMotorControl2(quad, LEGS[i][2], p.POSITION_CONTROL, angles[0])


def getAngles(leg, target_position, dir):
    r = 0.1
    target = -np.array([-1 * p.getJointInfo(quad, leg[0])[14][0], p.getJointInfo(quad, leg[0])[14][1],
                        p.getJointInfo(quad, leg[0])[14][2]]) + np.array(target_position)
    gama = math.atan(-target[1] / target[2]) - math.atan(p.getJointInfo(quad, leg[1])[14][1] / np.linalg.norm(target))
    if p.getJointInfo(quad, leg[0])[8] > gama:
        gama = p.getJointInfo(quad, leg[0])[8]
        target[1] = target[2] * math.tan(gama)
    elif p.getJointInfo(quad, leg[0])[9] < gama:
        gama = p.getJointInfo(quad, leg[0])[9]
        target[1] = target[2] * math.tan(gama)
    shoulder_position = np.array([dir[1] * 0.025, (0.03 * math.sin(gama) + dir[2] * 0.01 * math.cos(gama)),
                                  (-0.03 * math.cos(gama) + dir[2] * 0.01 * math.sin(gama))])
    target = target - shoulder_position
    leg_length = np.linalg.norm(target)
    fi = math.asin(target[0] / leg_length)

    if leg_length < 0.1931851653 and leg_length > r: #middle
        q = math.acos((-(leg_length ** 2)) / (2 * r * r) + 1)
        alpha = dir[0] * (math.pi - q)
        if (fi >= p.getJointInfo(quad, leg[1])[8] + alpha / 2 and fi <= p.getJointInfo(quad, leg[1])[9] + alpha / 2):
            beta = fi - alpha / 2 # VII
        elif fi < p.getJointInfo(quad, leg[1])[8] + alpha / 2:
            beta = p.getJointInfo(quad, leg[1])[8] # VIII
        else:
            beta = p.getJointInfo(quad, leg[1])[9]
            alpha += 1 * ((fi - alpha / 2) - beta) # IX
    elif leg_length >= 0.1931851653:#outer
        alpha = p.getJointInfo(quad, leg[2])[8]
        if fi >= p.getJointInfo(quad, leg[1])[8] + dir[0] * 0.2617993878 and fi <= p.getJointInfo(quad, leg[1])[9] + dir[0] * 0.2617993878:
            beta = fi - dir[0] * 0.2617993878 # I
        elif fi < p.getJointInfo(quad, leg[1])[8] + dir[0] * 0.2617993878:
            beta = p.getJointInfo(quad, leg[1])[8] # II
        else:
            beta = p.getJointInfo(quad, leg[1])[9] # III
    else:#inner
        alpha = p.getJointInfo(quad, leg[2])[9]
        if fi >= p.getJointInfo(quad, leg[1])[8] + dir[0] * 1.047197552 and fi <= p.getJointInfo(quad, leg[1])[9] + dir[0] * 1.047197552:
            beta = fi - dir[0] * 1.047197552 # IV
        elif fi <= p.getJointInfo(quad, leg[1])[8] + dir[0] * 1.047197552:
            beta = p.getJointInfo(quad, leg[1])[8] # V
        else:
            beta = p.getJointInfo(quad, leg[1])[9] # VI
    return [alpha, beta, gama]


def walking(movement, orientation, other): #[r, cr, h, v], [sl, incl, theta, fl], [step_height, spread]
    addition = (0, 2, 1, 3)
    # addition = (0, 0, 0, 0)
    d = math.copysign(1, movement[3])
    movement[3] = math.fabs(movement[3])
    change_by_velocity = 1.3 - (3380 * (movement[3] + 0.0135) / 627) ** 0.5
    # if change_by_velocity > 1:
    #     change_by_velocity = 1
    # else:
    #     if change_by_velocity < 0:
    #         change_by_velocity = 0
    horizontal_drift_coafitient = change_by_velocity * 0.02
    vertical_drift_coafitient = change_by_velocity * 0.007
    global process_position
    # global legs_process_positions
    global grounded_legs
    global process_state
    # global horizontal_angular_drift
    # grounded_legs = []
    # legs_process_positions %= 4
    process_position %= 4
    global positions
    global alt_positions
    horizontal_drift = -horizontal_drift_coafitient * math.cos((math.pi * (process_position + movement[3])) / 2)
    # horizontal_drift = -horizontal_drift_coafitient * math.cos((math.pi * (legs_process_positions[0] + movement[3])) / 2)
    for i in range(4):
        t = (addition[i] + process_position + movement[3]) % 4
        # t = (addition[i] + legs_process_positions[i] + movement[3]) % 4
        if 0 <= t < 1:
            Theta = (t - 0.5) * 2 * orientation[2]
            grounded_legs[i] = False
            R = movement[0]
            if movement[0] > 0.07 and ((i < 2 and t < 0.5) or (i >= 2 and t >= 0.5)):
                R = 0.07
            z = 0.1217395 * (0.410713 * math.sqrt(1 - (2 * t - 1)**2) + 2 * other[0] * t)
            x = R * (2.43479 * t - 1.217395)
            y = movement[1] * (1.5 * t - 0.75)
        else:
            Theta = -(t - 2.5) * 2 / 3 * orientation[2]
            grounded_legs[i] = True
            if movement[0] > 0.07:
                if i < 2:
                    x = -(0.028405883 + 0.40579833 * movement[0]) * t + 0.40579833 * movement[0] * 4 + 0.028405883
                else:
                    x = -(0.028405883 + 0.40579833 * movement[0]) * t + 0.40579833 * movement[0] + 0.028405883 * 4
            else:
                x = movement[0] * (-0.81159667 * t + 2.02899167)
            z = 0
            # y = movement[1] * (-0.166666667 * t + 0.416666667)
            y = movement[1] * (-0.5 * t + 1.25)
        if i >= 2:
            t -= 1
        if i % 2 == 0:
            spread_dir = -1
        else:
            spread_dir = 1
        z += -vertical_drift_coafitient * math.cos((math.pi * t) / 2)
        y += horizontal_drift
        pos = starting_positions[i] + np.array([x, y, z])
        ro = math.sqrt(pos[0]**2 + pos[1]**2)
        fi = math.atan(pos[1] / pos[0])
        front_lean_height = pos[0] * math.tan(orientation[3])
        if i > 1:
            ro = -ro
        pos[0] = ro * math.cos(fi + Theta)
        pos[1] = ro * math.sin(fi + Theta) - spread_dir * pos[2] * math.sin(other[1])
        alt_position = np.copy(pos)
        alt_position[2] -= movement[2] + (pos[2] - pos[2] * math.cos(other[1])) + front_lean_height
        pos[2] -= (movement[2] - pos[1] * math.tan(orientation[0])) + (pos[2] - pos[2] * math.cos(other[1])) +  \
                                                                                                front_lean_height
        pos[0] -= 0.15 * math.tan(orientation[1]) # patvarkyti
        positions[i] = pos
        alt_positions[i] = alt_position
    process_position += d * movement[3]
    # legs_process_positions = legs_process_positions + d * movement[3]


def walking2(movement, orientation, other): #[r, cr, h, v], [sl, incl, theta, fl], [step_height, spread]
    global legs_process_positions
    global last_zs
    global grounded_legs
    global indiv_additional_heights
    # global process_state
    global positions
    global alt_positions

    setMovementState(last_movement_properties[0][3], last_movement_properties[1][2])
    # correctIndividualHeights()
    movement[3] = abs(movement[3])
    change_by_velocity = 1.3 - (3380 * (movement[3] + 0.0135) / 627) ** 0.5
    for i in range(4):
        if legs_height_positions[i] != 0:
            grounded_legs[i] = False
            z = 0.1217395 * (0.410713 * math.sqrt(1 - (2 * legs_height_positions[i] - 1) ** 2) + 2 * other[0] * legs_height_positions[i]) + indiv_additional_heights[i]
        else:
            grounded_legs[i] = True
            z = 0
        if movement[0] > 0.07 and ((i < 2 and legs_process_positions[i] < 0) or (i >= 2 and legs_process_positions[i] >= 0)):
            R = 0.07
            # print(legs_process_positions[i])
        else:
            R = movement[0]
        x = R * 1.217395 * legs_process_positions[i]
        y = movement[1] * legs_process_positions[i] * 0.75
        if i % 2 == 0:
            spread_dir = -1
        else:
            spread_dir = 1
        z += spread_dir * change_by_velocity * 0.007 * math.sin((math.pi * drift_coaf) / 2)
        y += change_by_velocity * -0.02 * math.sin((math.pi * drift_coaf) / 2)
        pos = starting_positions[i] + np.array([x, y, z])
        ro = math.sqrt(pos[0]**2 + pos[1]**2)
        fi = math.atan(pos[1] / pos[0])
        front_lean_height = pos[0] * math.tan(orientation[3])
        if i > 1:
            ro = -ro
        pos[0] = ro * math.cos(fi + legs_process_positions[i] * orientation[2])
        pos[1] = ro * math.sin(fi + legs_process_positions[i] * orientation[2]) - spread_dir * pos[2] * math.sin(other[1])
        alt_position = np.copy(pos)
        alt_position[2] -= movement[2] + (pos[2] - pos[2] * math.cos(other[1])) + front_lean_height
        pos[2] -= (movement[2] - pos[1] * math.tan(orientation[0])) + (pos[2] - pos[2] * math.cos(other[1])) + front_lean_height
        pos[0] -= 0.15 * math.tan(orientation[1])
        positions[i] = pos
        alt_positions[i] = alt_position
        last_zs[i] = z


def setMovementState(sp, trn):
    global legs_process_positions
    global legs_height_positions
    global process_monocount
    global advance_process_table
    global drift_coaf
    # if round(sp, 4) < starting_speed:#todo
    #     sp = starting_speed
    if process_monocount + sp >= 1:
        # print()
        process_monocount = (process_monocount + sp) % 1
        # if round(sp, 4) > starting_speed or abs(trn) >= 0.01:#todo
        if round(sp, 4) > 0.01 and starting_speed > 0.1: # starting speed is used as start/stop walking parameter, need to fix names todo
        # if round(sp, 4) > 0.01:
            process = 1
        else:
            process = 2
        advanced_positions = getAdvancedProcess(advance_process_table[1], process)
        advance_process_table[0] = advance_process_table[1]
        advance_process_table[1] = advanced_positions.flatten()
    else:
        process_monocount = process_monocount + sp
    uncompressced = advance_process_table[0] + (advance_process_table[1] - advance_process_table[0]) * process_monocount
    legs_process_positions = 2 * (1.5 - uncompressced) / 3
    # print(legs_process_positions)
    drift_coaf = drift_table[0] + (drift_table[1] - drift_table[0]) * process_monocount
    for i in range(4):
        if advance_process_table[1][i] - advance_process_table[0][i] < 0:
            legs_height_positions[i] = process_monocount
        else:
            legs_height_positions[i] = 0.0


def getAdvancedProcess(pos, process):
    global drift_table
    for i in range(step_table.shape[1]):
        if np.array_equal(pos, step_table[:4, i]):
            drift_table[0] = drift_table[1]
            drift_table[1] = step_table[11 + process, i]
            return step_table[process * 4:(process + 1) * 4, i]
    return np.array([[0, 0, 0, 0], [0, 0, 0, 0]])


def correctIndividualHeights():#TODO
    global indiv_additional_heights
    global indiv_air_tick_counts
    global indiv_air_tick_threshold
    global indiv_heights_suspended
    global positions
    global alt_positions
    global test_stop

    for i in range(4):
        if legs_height_positions[i] == 0:
            if touch_force[i] < touch_threshold:
                indiv_additional_heights[i] -= 0.005
        else:
            indiv_additional_heights[i] += min(0.005, max(-indiv_additional_heights[i], -0.005))
            if touch_force[i] > touch_threshold:
                indiv_additional_heights[i] = last_zs[i]
                print(last_zs[i])
    print(indiv_additional_heights)


def getLegsNormal(arr):
    has_normal = False
    for i in range(4):
        if not grounded_legs[clock_wise_sequence[i]] or not grounded_legs.__contains__(False):
            a = np.array(arr[clock_wise_sequence[i - 1]]) - np.array(
                arr[clock_wise_sequence[i - 2]])
            b = np.array(arr[clock_wise_sequence[i - 1]]) - np.array(
                arr[clock_wise_sequence[i - 3]])
            c = getCrossProduct(b, a) * np.array([-1, -1, 1])
            return np.array([1, -1, 1]) * streachVectorTo(c, 1)
    if not has_normal:
        return np.array([0, 0, -1])


def getVectorsCosine(a, b):
    if np.linalg.norm(a) * np.linalg.norm(b) == 0:
        return 0
    else:
        return (np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b))


def getCrossProduct(a, b):
    return np.cross(np.array(a), np.array(b))
    # return np.array([a[1] * b[2] - a[2] * b[1],  a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def getRotationMatrixFromTwoVectors(a, b):
    a = streachVectorTo(a, 1)
    b = streachVectorTo(b, 1)
    phi = math.acos(getVectorsCosine(a, b))
    axis = streachVectorTo(getCrossProduct(a, b), 1)
    matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    rcos = math.cos(phi)
    rsin = math.sin(phi)
    matrix[0][0] = rcos + axis[0] * axis[0] * (1 - rcos)
    matrix[1][0] = axis[2] * rsin + axis[1] * axis[0] * (1 - rcos)
    matrix[2][0] = -axis[1] * rsin + axis[2] * axis[0] * (1 - rcos)
    matrix[0][1] = -axis[2] * rsin + axis[0] * axis[1] * (1 - rcos)
    matrix[1][1] = rcos + axis[1] * axis[1] * (1 - rcos)
    matrix[2][1] = axis[0] * rsin + axis[2] * axis[1] * (1 - rcos)
    matrix[0][2] = axis[1] * rsin + axis[0] * axis[2] * (1 - rcos)
    matrix[1][2] = -axis[0] * rsin + axis[1] * axis[2] * (1 - rcos)
    matrix[2][2] = rcos + axis[2] * axis[2] * (1 - rcos)
    return matrix


def streachVectorTo(v, lenght):
    if np.linalg.norm(v) != 0:
        return v * (lenght / np.linalg.norm(v))
    else:
        return np.array([0, 0, 0])


def vectorProjection(from_vec, to_vec):
    return (np.dot(from_vec, to_vec)) / np.linalg.norm(to_vec)


def inertializeData(data_container, newdata, coafitient):
    newdata = newdata.tolist()
    if len(data_container) >= coafitient:
        data_container.pop(0)
    data_aver = np.array(newdata)
    for i in range(len(data_container)):
        data_aver += np.array(data_container[i])
    data_container.append(newdata)
    return data_aver / len(data_container)


def getAccelerationInfo(new_position):
    global old_position
    global old_velocity
    gravity = np.array([0, 0, -9.81])
    new_velocity = 20 * (new_position - old_position)
    old_position = new_position.copy()
    accel = gravity + new_velocity - old_velocity
    old_velocity = new_velocity.copy()
    return accel * np.array([-1, -1, 1])


def getBaseFrameOrientationMatrix():
    global horizontal_turn_matrix
    for i in range(4):
        if grounded_legs[clock_wise_sequence[i]] and grounded_legs[clock_wise_sequence[i - 1]] and \
                grounded_legs[clock_wise_sequence[i - 2]]:
            # a = np.array(positions[clock_wise_sequence[i]]) - np.array(
            #     positions[clock_wise_sequence[i - 1]])
            # b = np.array(positions[clock_wise_sequence[i]]) - np.array(
            #     positions[clock_wise_sequence[i - 2]])
            # c = getCrossProduct(b, a) * np.array([-1, -1, 1])
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


def shockControl():
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
        # if shock_control_timer == timer_minus_one:
        #     shock_control_timer += 1
        # else:
        #     if shock_control_timer == 0:
        #         shock_control_timer = 5
        av_map = av_map / (av_count * max_coordinate)
        return [1, np.array([-1, 1]) * np.array([av_map[0], av_map[1]]), force_container]
    elif not mid_air:
        return [0, np.array([0, 0]), force_container]
    else:
        return [-1, np.array([0, 0]), force_container]


def balanceControl():#returns: 0 - balsnce vector; 1 - is out of perimeter; 2 - cosine to X axis; 3 - cosine to Y axis; 4 - leg normal
    x_inversion = np.array([-1, 1, 1])

    gr_count = 0
    mid_air = True
    for i in range(4):

        if grounded_legs[i]:
            gr_count += 1
        if touch_force[i] > 0:
            mid_air = False
    if mid_air:
        return np.array([0, 0, 0, 0, 0])
    if gr_count == 2:
        return np.array([-3, 0, 0, 0, 0])
        # if touch_force[0] != 0 and grounded_legs[0] and touch_force[3] != 0 and grounded_legs[3]:
        # # if touch_force[0] != 0 and touch_force[3] != 0:
        # # if grounded_legs[0] and grounded_legs[3]:
        #     cp = getCrossProduct(positions[0], positions[3])
        #     # print(-streachVectorByRatio(cp))
        #     return -streachVectorByRatio(cp)
        # else:
        #     if touch_force[1] != 0 and grounded_legs[1] and touch_force[2] != 0 and grounded_legs[2]:
        #         cp = getCrossProduct(positions[1], positions[2])
        #         # print(-streachVectorByRatio(cp))
        #         return -streachVectorByRatio(cp)
        #     else:
        #         for i in range(4):
        #             if touch_force[clock_wise_sequence[i]] != 0 and grounded_legs[clock_wise_sequence[i]
        #             ] and touch_force[clock_wise_sequence[i - 1]
        #             ] != 0 and grounded_legs[clock_wise_sequence[i - 1]]:
        #                 cp = getCrossProduct(positions[i], positions[i - 1])
        #                 if getVectorsCosine(cp, base_force_vector) > 0:
        #                     return -streachVectorByRatio(cp)
        #                 else:
        #                     return np.array([0, 0, 0])
    else:
        if gr_count > 2:
            out_of_perimeter = False
            c = getLegsNormal(positions)
            # p.addUserDebugLine(up, up + c, [1, 0, 0], 2, 0.1)
            # print(grounded_legs)
            # p.addUserDebugLine(up, up + -c, [0, 0, 0], 2, 0.1)
            for i in range(4):
                if grounded_legs[clock_wise_sequence[i]]:
                    if grounded_legs[clock_wise_sequence[i - 1]]:
                        other = i - 1
                    else:
                        other = i - 2
                    cp = getCrossProduct(x_inversion * positions[clock_wise_sequence[i]],
                                         x_inversion * positions[clock_wise_sequence[other]])
                    cp = streachVectorTo(cp, 0.5)

                    if getVectorsCosine(cp, x_y_inversion * acceleration) > 0:
                        out_of_perimeter = True
                        break
            c = streachVectorTo(c, vectorProjection(x_y_inversion * acceleration, c))
            # print(np.linalg.norm(x_y_inversion * acceleration - c), math.acos(getVectorsCosine(np.array([-1, 0, 1]) * acceleration, np.array([1, 0, 1]) * c)), math.acos(getVectorsCosine(np.array([0, -1, 1]) * acceleration, np.array([0, 1, 1]) * c)))

            return [x_y_inversion * acceleration - c, out_of_perimeter, getVectorsCosine(np.array([-1, 0, 1]) *
                acceleration, np.array([1, 0, 1]) * c), getVectorsCosine(np.array([0, -1, 1]) *
                                                                acceleration, np.array([0, 1, 1]) * c), c]
        else:
            return np.array([0, 0, 0, 0, 0])


def makeMovementArgsLite():
    global air_tick_count
    global after_air_tick_count
    # global shock_tick_count
    global instability_timer
    global last_movement_properties
    global last_velocity_properties
    global base_force_vector_container
    # global longwise_count
    global rebound_container
    global starting_speed

    # global plot_time_left

    s = shockControl()
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
    if after_air_tick_count >= 0:
        after_air_tick_count -= 1
    if s[0] == -1:
        air_tick_count += 1
    else:
        air_tick_count = 0
    if air_tick_count >= 3:
        stand_dir = np.array([-1, -1, 1]) * base_frame_orientation_matrix.dot(np.array([0, 0, 1]))
        # speed = 0
        starting_speed = 0
        turn = 0
        step_height = 0
        side_step = 0
        after_air_tick_count = 5
        diff = ((-stand_dir[0] / stand_dir[2]) - incl) * 0.4
        rebound_container[3] = checkRestricionsReb(rebound_container[3], incl, diff, 5)
        incl += diff
        diff = (abs(stand_dir[1] / stand_dir[2]) - spread) * 0.2
        rebound_container[4] = checkRestricionsReb(rebound_container[4], spread, diff, 9)
        spread += diff

        print("in air")
        if last_movement_properties[0][2] + 0.01 < 0.02:
            diff = 0.01
            rebound_container[0] = checkRestricionsReb(rebound_container[0], h, diff, 2)
            h += diff
        else:
            diff = 0.02 - last_movement_properties[0][2]
            rebound_container[0] = checkRestricionsReb(rebound_container[0], h, diff, 2)
            h += diff
        if abs(last_movement_properties[1][3]) != 0:
            diff = -0.1 * math.atan(last_movement_properties[1][3] * 10)
            rebound_container[1] = checkRestricionsReb(rebound_container[1], fl, diff, 7)
            fl += diff
        if abs(last_movement_properties[1][0]) != 0:
            diff = -0.05 * math.atan(last_movement_properties[1][0] * 20)
            rebound_container[2] = checkRestricionsReb(rebound_container[2], sl, diff, 4)
            sl += diff
    elif s[0] == 1:
        # starting_speed = param2
        f_force = s[2] * s[1][0]
        s_force = s[2] * s[1][1]
        stand_dir = np.array([-1, -1, 1]) * base_frame_orientation_matrix.dot(np.array([0, 0, -1]))

        diff = ((-stand_dir[0] / stand_dir[2]) - incl) * 0.4
        rebound_container[3] = checkRestricionsReb(rebound_container[3], incl, diff, 5)
        incl += diff
        diff = (abs(stand_dir[1] / stand_dir[2]) - spread) * 0.2
        rebound_container[4] = checkRestricionsReb(rebound_container[4], spread, diff, 9)
        spread += diff

        diff = 0.1 * f_force / shock_threshold
        rebound_container[1] = checkRestricionsReb(rebound_container[1], fl, diff, 7)
        fl += diff

        diff = 0.1 * s_force / shock_threshold
        rebound_container[2] = checkRestricionsReb(rebound_container[2], sl, diff, 4)
        sl += diff

        diff = 0.05 * ((base_force_vector[2] / base_shock_threshold) + 1)
        rebound_container[0] = checkRestricionsReb(rebound_container[0], h, diff, 2)
        h += diff
        print("shock", touch_force)
    else:
        # starting_speed = param2
        rebounding_consts = np.array([0.2, 0.2, 0.25, 0.2, 0.2])  #  h fl sl incl spread
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


    last_movement_properties = np.array([[f_step, side_step, h, speed], [sl, incl, turn, fl], [step_height, spread, 0, 0]])
    checkRestrictionsUniversal(last_movement_properties)
    # print(last_movement_properties)
    # setMovementState(last_movement_properties[0][3], last_movement_properties[1][2])
    walking2(last_movement_properties[0], last_movement_properties[1], last_movement_properties[2])


def makeMovementArgs(velocity, turn, side_step, step_height):
    global air_tick_count
    global after_air_tick_count
    global shock_tick_count
    global instability_timer
    global last_movement_properties
    global last_velocity_properties
    global base_force_vector_container
    global longwise_count
    # global plot_time_left

    s = shockControl()
    f_step = last_movement_properties[0][0]
    fl = last_movement_properties[1][3]
    sl = last_movement_properties[1][0]
    incl = last_movement_properties[1][1]
    h = last_movement_properties[0][2]
    speed = 0
    spread = math.pi / 30
    if shock_tick_count < 5:
        shock_tick_count += 1
    if after_air_tick_count >= 0:
        after_air_tick_count -= 1
    if longwise_count <= 11:
        longwise_count += 1
    if s[0] == -1:
        air_tick_count += 1
    else:
        air_tick_count = 0
    if air_tick_count >= 5:
        stand_dir = np.array([-1, -1, 1]) * base_frame_orientation_matrix.dot(np.array([0, 0, 1]))
        speed = 0
        turn = 0
        step_height = 0
        side_step = 0
        after_air_tick_count = 5
        incl = -stand_dir[0] / stand_dir[2]
        spread = abs(stand_dir[1] / stand_dir[2])

        print("in air")
        if last_movement_properties[0][2] + 0.01 < 0.02:
            h += 0.01
        else:
            h += 0.02 - last_movement_properties[0][2]
        if abs(last_movement_properties[1][3]) != 0:
            fl -= 0.1 * math.atan(last_movement_properties[1][3] * 10)
        if abs(last_movement_properties[1][0]) != 0:
            sl -= 0.05 * math.atan(last_movement_properties[1][0] * 20)
    else:
        longwise_count = 0
        if s[0] == 1 and after_air_tick_count > 0:
            shock_tick_count = 0
            # p.addUserDebugLine(up, up + [s[1][0], s[1][1], 1], [0, 1, 1], 2, 0.05)
            f_force = s[2] * s[1][0]
            s_force = s[2] * s[1][1]
            stand_dir = np.array([-1, -1, 1]) * base_frame_orientation_matrix.dot(np.array([0, 0, -1]))
            incl = -stand_dir[0] / stand_dir[2]
            spread += abs(stand_dir[1] / stand_dir[2])
            # side_step = last_movement_properties[0][1]
            # speed = last_movement_properties[0][3]
            # turn = last_movement_properties[1][2]
            # step_height = last_movement_properties[2][0]
            fl += 0.1 * f_force / shock_threshold
            sl += 0.1 * s_force / shock_threshold
            h += 0.05 * ((base_force_vector[2] / base_shock_threshold) + 1)
            print("shock", touch_force)
        else:
            b = balanceControl()
            ln = -getLegsNormal(alt_positions)
            side_leaning = math.atan(-acceleration[1] / acceleration[2])
            front_leaning = math.atan(-acceleration[0] / acceleration[2])

            if abs(last_movement_properties[0][2]) != 0:
                h -= 0.01 * math.atan(last_movement_properties[0][2] * 100)
            if abs(last_movement_properties[1][1]) != 0:
                incl -= 0.02 * math.atan(last_movement_properties[1][1] * 100)
            # if abs(last_movement_properties[1][3]) != 0:
            #     fl -= 0.1 * math.atan(last_movement_properties[1][3] * 10)
            if abs(last_movement_properties[1][0]) != 0:
                if shock_tick_count < 5:
                    sl -= 0.05 * math.atan(last_movement_properties[1][0] * 20)
                # else:
                #     sl -= 0.02 * math.atan(last_movement_properties[1][0] * 20)
            # else:
            # if np.linalg.norm(b[0]) >= 1:
            # print(math.acos(b[3]), l)
            if math.acos(b[3]) >= math.pi / 30:
                # print("Y")
                if (touch_force[0] or touch_force[2]) and (
                        touch_force[1] or touch_force[3]):
                    sl -= 0.1 * math.tanh((math.atan(ln[1] / ln[2]) - side_leaning) * 2)
                else:
                    if ((touch_force[0] or touch_force[2]) and acceleration[1] < 0) or (
                            (touch_force[1] or touch_force[3]) and acceleration[1] > 0):
                        sl += 0.02 * math.tanh((math.atan(ln[1] / ln[2]) - side_leaning) * 2)
            else:
                sl -= 0.02 * math.atan(last_movement_properties[1][0] * 20)
            # if np.linalg.norm(b[0]) >= 1:
            if math.acos(b[2]) >= math.pi / 20:
                # print("X")
                if (touch_force[0] or touch_force[1]) and (
                        touch_force[2] or touch_force[3]):
                    incl -= math.tan((last_movement_properties[1][1] + front_leaning))
                    longwise_count = 0
                elif (((touch_force[0] or touch_force[1]) and acceleration[0] > 0) or (
                            (touch_force[2] or touch_force[3]) and acceleration[0] < 0)) \
                            and longwise_count == 11:
                    incl -= math.tan((last_movement_properties[1][1] + front_leaning))
                    fl -= math.tan((last_movement_properties[1][3] + front_leaning))
                    checkRestrictionsUniversal(np.array([fl, incl]), tuple([7, 7]))
            else:
                # incl -= 0.02 * math.atan(last_movement_properties[1][1] * 100)
                fl -= 0.1 * math.atan(last_movement_properties[1][3] * 10)

                # incl += 0.1 * math.atan(last_movement_properties[1][3] * 10)
                # fl += 0.1 * math.atan(last_movement_properties[1][3] * 10)
                # print(math.tan((last_movement_properties[1][1] + front_leaning)))

            if b[1] or np.linalg.norm(b[0]) >= 2.5:
                instability_timer += 1
                base_force_vector_container = base_force_vector_container + b[0]
            else:
                instability_timer = 0
                base_force_vector_container = np.array([0, 0, 0])

            if abs(velocity) >= 0.005 or abs(side_step) >= 0.001 or abs(turn) > 0.01:
                # f_speed = 0.25 + abs(velocity) / 6
                # s_speed = 0.25 + abs(side_step) / 2
                f_speed = starting_speed + abs(velocity) * (0.3 - starting_speed) / 0.3
                s_speed = starting_speed + abs(side_step) * (0.3 - starting_speed) * 3 / 0.3
                if f_speed >= s_speed:
                    speed_0 = f_speed
                    f_step_0 = velocity / 3
                    # f_step_0 = 0.1 / (1 + math.e ** (-30 * (velocity - 0.15)))
                    # print(f_step_0)
                    side_step_0 = (s_speed * side_step) / speed_0
                else:
                    speed_0 = s_speed
                    side_step_0 = side_step
                    f_step_0 = f_speed * (velocity / 3) / speed_0
                # print(f_speed)
            else:
                speed_0 = f_step_0 = side_step_0 = 0
            if instability_timer >= 2 and (
                    np.linalg.norm(base_force_vector_container) >= 4 or np.linalg.norm(b[0]) >= 2):

                v = np.delete(b[0], 2) * 0.02 + np.array([
                    (speed_0 + last_velocity_properties[0]) * (f_step_0 + last_velocity_properties[1]) * 0.405808,
                    (speed_0 + last_velocity_properties[0]) * (side_step_0 + last_velocity_properties[2]) * 0.25])\
                    / 0.02
            else:
                v = np.array([
                    (speed_0 + last_velocity_properties[0]) * (f_step_0 + last_velocity_properties[1]) * 0.405808,
                    (speed_0 + last_velocity_properties[0]) * (side_step_0 + last_velocity_properties[2]) * 0.25])\
                    / 0.02

            vel = v[0] * 0.4928439
            ss = v[1] * (4 / 15)
            # vel = v[0] * 0.4928439 * 1
            # ss = v[1] * (4 / 15)
            # print(vel, ss)
            if abs(vel) >= 0.006 or abs(ss) >= 0.008 or abs(turn) > 0.01:
                # f_speed = 0.25 + abs(vel) / 6
                # s_speed = 0.25 + abs(ss) / 2
                f_speed = starting_speed + abs(velocity) * (0.3 - starting_speed) / 0.3
                s_speed = starting_speed + abs(side_step) * (0.3 - starting_speed) * 3 / 0.3
                # print(f_speed)
                if f_speed >= s_speed:
                    speed = f_speed
                    f_step = (vel / 3)
                    # f_step = 0.1 * math.tanh(5*vel)
                    # f_step = 0.1 / (1 + math.e ** (-30 * (vel - 0.15)))
                    # print(vel, f_step)
                    # f_step = 0.1 * math.pow(10.0 / 3.0 * vel, 1/3)
                    # f_step = vel
                    side_step = ((s_speed * ss) / speed)
                else:
                    speed = s_speed
                    side_step = ss
                    # f_step = ((f_speed * vel) / speed)
                    f_step = ((f_speed * (vel / 3)) / speed)
                if speed != 0:
                    last_velocity_properties = np.array([speed - speed_0, f_step - f_step_0, side_step - side_step_0])
                    spread += -(math.pi / 8) * math.cos(10 * math.pi * last_velocity_properties[2]) + math.pi / 8
                    checkRestrictionsUniversal(last_velocity_properties, (3, 0, 1))
                else:
                    last_velocity_properties = np.array([0, 0, 0])
            # last_velocity_properties = np.array([speed - speed_0, f_step - f_step_0, side_step - side_step_0])
            # spread += -(math.pi / 18) * math.cos(10 * math.pi * last_velocity_properties[2]) + math.pi / 18
            # checkVelocityRestrictions(last_velocity_properties)
            else:
                last_velocity_properties = np.array([0, 0, 0])
            # spread += -(math.pi / 18) * math.cos(10 * math.pi * last_velocity_properties[2]) + math.pi / 18
            last_velocity_properties = last_velocity_properties * 0.75
            # if plot_time_left <= 1000:
            #     plot_time_left += 1
            #     if plot_time_left == 1000:
            #         df = pd.DataFrame(
            #             {'x': range(999), 'y1': accel_list, 'y2': vel_list})
            #
            #         # multiple line plot
            #         plt.plot('x', 'y1', data=df, marker='', color='blue', linestyle='solid', label="accel")
            #         plt.plot('x', 'y2', data=df, marker='', color='red', linestyle='solid', label="vel")
            #
            #         plt.legend()
            #         plt.grid()
            #         plt.show()
            #     else:
            #
            #         accel_list.append(acceleration[1] * 0.02)
            #         vel_list.append(v[1])

    # print(last_movement_properties[0][3])
    last_movement_properties = np.array([[f_step, side_step, h, speed], [sl, incl, turn, fl], [step_height, spread, 0, 0]])
    checkRestrictionsUniversal(last_movement_properties)
    setMovementState(last_movement_properties[0][3], last_movement_properties[1][2])
    walking2(last_movement_properties[0], last_movement_properties[1], last_movement_properties[2])


def checkRestrictionsUniversal(prop, indices=tuple(range(10))):
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


def checkRestricionsReb(prop, current_value, addition, index):
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


p.connect(p.GUI)
plane = p.createCollisionShape(p.GEOM_PLANE)
obj1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.5, 0.04], collisionFramePosition=[0, 0.8, 0.025],
                                                                    collisionFrameOrientation=[0.08, 0, 0, 1])
obj2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.5, 0.04], collisionFramePosition=[0, 0.75, -0.015],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])
obj3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.04], collisionFramePosition=[-1, 0, -0.02],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])
obj4 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.04], collisionFramePosition=[-1.1, 0, -0.015],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])
obj5 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.04], collisionFramePosition=[-1.18, 0, -0.02],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])
obj6 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.04], collisionFramePosition=[-1.23, 0, -0.025],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])
obj7 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.04], collisionFramePosition=[-1.3, 0, -0.02],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])
obj8 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.04], collisionFramePosition=[-1.4, 0, -0.015],
                                                                    collisionFrameOrientation=[0, 0, 0, 1])

p.createMultiBody(0, obj1)
p.createMultiBody(0, obj2)
p.createMultiBody(0, obj3)
p.createMultiBody(0, obj4)
p.createMultiBody(0, obj5)
p.createMultiBody(0, obj6)
p.createMultiBody(0, obj7)
p.createMultiBody(0, obj8)
p.createMultiBody(0, plane)
p.setGravity(0, 0, -10)
useMaximalCoordinates = False

# quad = p.loadURDF("quad.xacro", [0, 0, 0], baseOrientation=p.getQuaternionFromEuler([0, 0.3, 0], 0), useFixedBase=True,
#                   useMaximalCoordinates=useMaximalCoordinates)
# quad = p.loadURDF("quad.xacro", [0, 0, 0], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)
quad = p.loadURDF("quad.xacro", [0.0, 0.0, 0.0], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
# Getting joints indices
nJoints = p.getNumJoints(quad)
jointNameToId = {}
for k in range(nJoints):
    jointInfo = p.getJointInfo(quad, k)
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

base_sensor = jointNameToId['base_sensor']
front_left_base_to_shoulder = jointNameToId['front_left_base_to_shoulder']
front_left_shoulder = jointNameToId['front_left_shoulder']
front_left_knee = jointNameToId['front_left_knee']
back_left_base_to_shoulder = jointNameToId['back_left_base_to_shoulder']
back_left_shoulder = jointNameToId['back_left_shoulder']
back_left_knee = jointNameToId['back_left_knee']
front_right_base_to_shoulder = jointNameToId['front_right_base_to_shoulder']
front_right_shoulder = jointNameToId['front_right_shoulder']
front_right_knee = jointNameToId['front_right_knee']
back_right_base_to_shoulder = jointNameToId['back_right_base_to_shoulder']
back_right_shoulder = jointNameToId['back_right_shoulder']
back_right_knee = jointNameToId['back_right_knee']
front_left_sensor = jointNameToId['front_left_sensor']
front_right_sensor = jointNameToId['front_right_sensor']
back_left_sensor = jointNameToId['back_left_sensor']
back_right_sensor = jointNameToId['back_right_sensor']

up = np.array([0, 0, 1])
x_y_inversion = np.array([-1, -1, 1])

FRONT_DIR_LEFT = [1, -1, -1]
FRONT_DIR_RIGHT = [1, -1, 1]
BACK_DIR_LEFT = [-1, 1, -1]
BACK_DIR_RIGHT = [-1, 1, 1]
DIRECTIONS = [FRONT_DIR_LEFT, FRONT_DIR_RIGHT, BACK_DIR_LEFT, BACK_DIR_RIGHT]

FRONT_LEFT = [front_left_base_to_shoulder, front_left_shoulder, front_left_knee]
FRONT_RIGHT = [front_right_base_to_shoulder, front_right_shoulder, front_right_knee]
BACK_LEFT = [back_left_base_to_shoulder, back_left_shoulder, back_left_knee]
BACK_RIGHT = [back_right_base_to_shoulder, back_right_shoulder, back_right_knee]
LEGS = [FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT]
SENSORS = [front_left_sensor, front_right_sensor, back_left_sensor, back_right_sensor, base_sensor]

restrictions = np.array([
    [-0.1, -0.1, -0.04, -0.15, -0.5, -0.8, -math.pi / 6, -math.pi / 9, -1, math.pi / 60],
    [0.1, 0.1, 0.04, 0.3, 0.5, 0.8, math.pi / 6, math.pi / 9, 1, math.pi / 6]]
)

step_table = np.array([
    [0,   1,   2,   3,   1,   2,   0.5, 1.5, 2.5, 1.5],
    [2,   3,   0,   1,   1,   2,   2.5, 1.5, 0.5, 1.5],
    [1,   2,   3,   0,   2,   1,   1.5, 2.5, 1.5, 0.5],
    [3,   0,   1,   2,   2,   1,   1.5, 0.5, 1.5, 2.5],

    [1,   2,   3,   0,   1.5, 2.5, 1,   2,   3,   0],
    [3,   0,   1,   2,   1.5, 0.5, 3,   0,   1,   2],
    [2,   3,   0,   1,   2.5, 1.5, 2,   3,   0,   1],
    [0,   1,   2,   3,   0.5, 1.5, 0,   1,   2,   3],

    [0.5, 1.5, 2.5, 1.5, 1,   0.5, 1,   2,   1,   2],
    [2.5, 1.5, 0.5, 1.5, 1,   2.5, 1,   2,   1,   2],
    [1.5, 2.5, 1.5, 0.5, 2,   1.5, 2,   1,   2,   1],
    [1.5, 0.5, 1.5, 2.5, 2,   1.5, 2,   1,   2,   1],

    [-1,  0,   1,   0,  -1,  -1,  -1,   0,   1,   0],
    [-1,  0,   1,   0,   0,   1,   0,   0,   0,   0]
])

acceleration = np.array([0, 0, 0], dtype=np.float)
advance_process_table = np.array([[1.0, 1.0, 2.0, 2.0], [1.0, 1.0, 2.0, 2.0]])
after_air_tick_count = 5
air_tick_count = 11
alt_positions = np.array([[0.0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
base_force_vector = np.array([0, 0, 0], dtype=np.float)
base_force_vector_container = np.array([0, 0, 0], dtype=np.float)
base_frame_orientation_matrix = np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]])
base_orientation = np.array(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(quad)[1]))
base_orientation_matrix = np.zeros((3, 3))
base_shock_threshold = 2.5
base_position = np.array(p.getBasePositionAndOrientation(quad)[0])
clock_wise_sequence = [0, 1, 3, 2]
cross_sequence = [0, 3, 1, 2]
drift_coaf = 0
drift_table = np.array([0.0, 0.0])
grounded_legs = np.array([False, False, False, False])
horizontal_turn_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
indiv_air_tick_counts = np.array([0, 0, 0, 0])
indiv_air_tick_threshold = 3
indiv_additional_heights = np.array([0, 0, 0, 0])
indiv_heights_suspended = np.array([False, False, False, False])
instability_timer = 0
last_movement_properties = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
last_velocity_properties = np.array([0, 0, 0])
last_zs = np.array([0.0, 0.0, 0.0, 0.0])
legs_height_positions = np.array([0.0, 0.0, 0.0, 0.0])
legs_process_positions = np.array([0, 0, 0, 0])
longwise_count = 10
old_position = np.array([0, 0, 0])
old_velocity = np.array([0, 0, 0]) # for accelerometer
rebound_container = np.zeros(5)
shock_threshold = 37.8 * 1
touch_threshold = shock_threshold / 16
# touch_threshold = shock_threshold / 8
shock_tick_count = 0
starting_positions = np.array([[0.125, -0.065, -0.24], [0.125, 0.065, -0.24], [-0.125, -0.065, -0.24], [-0.125, 0.065, -0.24]])
starting_speed = 0.25
# starting_speed = 0.15
plot_time_left = 0
positions = np.array([[0.0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
process_monocount = 0.0
process_position = 0.0
process_state = 0
touch_force = [0, 0, 0, 0]
touch_force_max = [0, 0, 0, 0]
# touch_force_queues = [queue.Queue(5), queue.Queue(5), queue.Queue(5), queue.Queue(5)]


test_stop = False
test_counter = -1
test_counter2 = -1
timer_running = False

p.setRealTimeSimulation(0)

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
# param = p.addUserDebugParameter("parameter", 0.02, 2, 0.02)
param2 = p.addUserDebugParameter("parameter2", 0, 0.25, 0.1)

# for j in range(0, 17):
#     print(p.getJointInfo(quad, j)[1])

for j in range(-1, 17):
    p.changeDynamics(quad, j, lateralFriction=2)
    print(p.getDynamicsInfo(quad, j))
for j in range(0, 17, 4):
    print(p.getJointInfo(quad, j)[1])
    p.enableJointForceTorqueSensor(quad, j, 1)


toStartingPosition()
# mainTimeManager()
# getCommands()
# timeManger()
# timeManger2()
counter = 0


# print(getRotationMatrixFromTwoVectors(streachVectorTo(np.array([0.2, 0, 0]), 1), streachVectorTo(np.array([0, 2, 0]), 1)))
def high_freq():
    global touch_force_queues
    p.stepSimulation()
    for i in range(4):
        # touch_force_queues[i].put(-p.getJointState(quad, SENSORS[i])[2][2])

        touch_force_max[i] = max(-p.getJointState(quad, SENSORS[i])[2][2], touch_force_max[i])

while 1:
    old = time.perf_counter_ns()
    high_freq()
    if counter % 5 == 0:
        t = threading.Thread(target=getCommands)
        t.start()
    counter += 1
    new = time.perf_counter_ns()

    # time.sleep(1 / 240 * 50 * p.readUserDebugParameter(param))
    time.sleep(1/250)
    # time.sleep(abs(1/250 - (new - old) / 1000000000))
    # time.sleep(1/60)
    # time.sleep(abs(1 / 250 * 50 * p.readUserDebugParameter(param) - (new - old) / 1000000000))
# while 1:
#     p.stepSimulation()
#     time.sleep(1 / 240)