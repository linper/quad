from leg import Leg
from consts import *
import pybullet as p
import numpy as np
import math

class SensInfo():
    def __init__(self):
        self.base_position = None
        self.base_force_vector = None
        self.touch_force = None
        self.base_orientation_matrix = None
        self.base_frame_orientation_matrix = None
        self.base_orientation = None
        self.base_position = None
        self.horizontal_turn_matrix = None

class Quad:
    def __init__(self, model, fll, frl, bll, brl, sensor):
        self.model = model
        self.sens_info = SensInfo()
        self.sensor = sensor
        self.front_left_leg: Leg = fll
        self.front_right_leg: Leg = frl
        self.back_left_leg: Leg = bll
        self.back_right_leg: Leg = brl

        self.legs = [fll, frl, bll, brl]
        self.sensors = [fll.sensor, frl.sensor, bll.sensor, brl.sensor, sensor]

        for i, l in enumerate(self.legs):
            l.idx = i
            l.body = self

    # # some math, nothing to see here
    def get_base_frame_orientation_matrix(self):
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
                self.sens_info.horizontal_turn_matrix = horizontal_turn_m.copy()
                new_matrix = np.matmul(horizontal_turn_m, base_orientation_matrix)
                return new_matrix
        return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # gets info from sensors
    def update_sensor_info(self):
        self.sens_info.base_position = np.array(p.getBasePositionAndOrientation(self.model)[0])
        self.sens_info.base_force_vector = -np.array(p.getJointState(self.model, self.sensor)[2][slice(3)])
        self.sens_info.base_orientation = np.array(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.model)[1]))
        base_orientation_1d = np.array(p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(self.model)[1]))
        for i in range(9):
            base_orientation_matrix[i // 3][i % 3] = base_orientation_1d[i]
        self.sens_info.base_frame_orientation_matrix = self.get_base_frame_orientation_matrix()
        self.sens_info.touch_force = touch_force_max
        for i in range(4):
            touch_force_max[i] = max(-p.getJointState(self.model, self.sensors[i])[2][2], touch_force_max[i])

    def to_starting_position(self):
        # global positions
        # positions = starting_positions.copy()
        p.setJointMotorControl2(self.model, self.front_left_leg.base, p.POSITION_CONTROL, -0.08354599985540861)
        p.setJointMotorControl2(self.model, self.front_left_leg.shoulder, p.POSITION_CONTROL, -0.7116395204323857)
        p.setJointMotorControl2(self.model, self.front_left_leg.knee, p.POSITION_CONTROL, 1.423279103830849)
        # p.setJointMotorControl2(self.model, self.front_left_leg.heel, p.POSITION_CONTROL, -1.423279103830849)
        p.setJointMotorControl2(self.model, self.back_left_leg.base, p.POSITION_CONTROL, -0.08354599985540861)
        p.setJointMotorControl2(self.model, self.back_left_leg.shoulder, p.POSITION_CONTROL, 0.7116395204323857)
        p.setJointMotorControl2(self.model, self.back_left_leg.knee, p.POSITION_CONTROL, -1.423279103830849)
        # p.setJointMotorControl2(self.model, self.back_left_leg.heel, p.POSITION_CONTROL, 1.423279103830849)
        p.setJointMotorControl2(self.model, self.front_right_leg.base, p.POSITION_CONTROL, 0.08354599985540861)
        p.setJointMotorControl2(self.model, self.front_right_leg.shoulder, p.POSITION_CONTROL, -0.7116395204323857)
        p.setJointMotorControl2(self.model, self.front_right_leg.knee, p.POSITION_CONTROL, 1.423279103830849)
        # p.setJointMotorControl2(self.model, self.front_right_leg.heel, p.POSITION_CONTROL, -1.423279103830849)
        p.setJointMotorControl2(self.model, self.back_right_leg.base, p.POSITION_CONTROL, 0.08354599985540861)
        p.setJointMotorControl2(self.model, self.back_right_leg.shoulder, p.POSITION_CONTROL, 0.7116395204323857)
        p.setJointMotorControl2(self.model, self.back_right_leg.knee, p.POSITION_CONTROL, -1.423279103830849)
        # p.setJointMotorControl2(self.model, self.back_right_leg.heel, p.POSITION_CONTROL, 1.423279103830849)