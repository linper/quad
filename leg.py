import math

import pybullet as p
from fsm import FSM, FSMState
from plan import *
from plan import DestPoint


class Leg:
    def __init__(self, name, base, shoulder, knee, heel, damp, sensor, dir, pos, off):
        self.next = None
        self.prev = None
        self.idx = -1
        self.name = name
        self.body = None
        self.plan = Plan(self)
        self.fsm = FSM(self)
        self.base = base
        self.shoulder = shoulder
        self.knee = knee
        self.heel = heel
        self.dampener = damp
        self.sensor = sensor
        self.position = np.array(pos)
        self.def_pos = np.array(pos)
        self.base_off = np.array(off)
        self.grounded = True
        self.dir = dir
        self.sh_w = 0.02
        self.sh_h = 0.03
        self.link_len = 0.1
        self.damp_len = 0.012
        self.stiffness_c = 2.5

    def get_angles(self) -> list:
        target = np.copy(self.position) - self.base_off

        base_st = p.getJointInfo(self.body.model, self.base)
        shoulder_st = p.getJointInfo(self.body.model, self.shoulder)
        knee_st = p.getJointInfo(self.body.model, self.knee)

        gama = math.atan(-target[1] / target[2])
        if base_st[8] > gama:
            gama = base_st[8]
            target[1] = -target[2] * math.tan(gama)
        elif base_st[9] < gama:
            gama = base_st[9]
            target[1] = -target[2] * math.tan(gama)
        shoulder_position = np.array([0.0, (0.03 * math.sin(gama) + self.dir[2] * 0.01 * math.cos(gama)),
                                      (-0.03 * math.cos(gama) + self.dir[2] * 0.01 * math.sin(gama))])
        target = target - shoulder_position
        leg_length = np.linalg.norm(target)
        max_leg_len = self.link_len * math.sqrt(5 - 4 * math.cos(abs(knee_st[9])))
        min_leg_len = self.link_len * math.sqrt(5 - 4 * math.cos(abs(knee_st[8])))

        if max_leg_len > leg_length > min_leg_len:  # middle
            e = math.acos(1.25 - ((leg_length ** 2) / (4 * self.link_len ** 2)))
            alpha = self.dir[0] * (math.pi - e)
        elif leg_length >= max_leg_len:  # outer
            alpha = knee_st[8]
        else:  # inner
            alpha = knee_st[9]

        fi = math.asin(target[0] / leg_length)
        theta = math.asin(min(1, max(-1, (self.link_len * math.sin(alpha)) / leg_length)))

        if shoulder_st[8] <= -fi + theta * -1 <= shoulder_st[9]:
            beta = -fi + theta * -1  # VII
        elif shoulder_st[8] > -fi + theta * -1:
            beta = shoulder_st[8]  # VIII
        else:
            beta = shoulder_st[9]

        return [alpha, beta, gama]

    # def get_angles(self) -> list:
    #     target = np.copy(self.position)
    #
    #     base_st = p.getJointInfo(self.body.model, self.base)
    #     shoulder_st = p.getJointInfo(self.body.model, self.shoulder)
    #     knee_st = p.getJointInfo(self.body.model, self.knee)
    #
    #     gama = math.atan(-target[1] / target[2])
    #     if base_st[8] > gama:
    #         gama = base_st[8]
    #         target[1] = target[2] * math.tan(gama)
    #     elif base_st[9] < gama:
    #         gama = base_st[9]
    #         target[1] = target[2] * math.tan(gama)
    #     shoulder_position = np.array([self.dir[1] * 0.025, (0.03 * math.sin(gama) + self.dir[2] * 0.01 * math.cos(gama)),
    #                                   (-0.03 * math.cos(gama) + self.dir[2] * 0.01 * math.sin(gama))])
    #     target = target - shoulder_position
    #     leg_length = np.linalg.norm(target)
    #     max_leg_len = self.link_len * math.sqrt(2 - 2 * math.cos(abs(knee_st[9])))
    #     min_leg_len = self.link_len * math.sqrt(2 - 2 * math.cos(abs(knee_st[8])))
    #     fi = math.asin(target[0] / leg_length)
    #
    #     if max_leg_len > leg_length > min_leg_len:   # middle
    #         e = math.acos((-(leg_length ** 2)) / (2 * self.link_len ** 2) + 1)
    #         alpha = self.dir[0] * (math.pi - e)
    #         if shoulder_st[8] + alpha / 2 <= fi <= shoulder_st[9] + alpha / 2:
    #             beta = fi - alpha / 2  # VII
    #         elif fi < shoulder_st[8] + alpha / 2:
    #             beta = shoulder_st[8]  # VIII
    #         else:
    #             beta = shoulder_st[9]
    #             alpha += 1 * ((fi - alpha / 2) - beta)  # IX
    #     elif leg_length >= max_leg_len:  # outer
    #         alpha = knee_st[8]
    #         if shoulder_st[8] + self.dir[0] * 0.2617993878 <= fi <= \
    #                 shoulder_st[9] + self.dir[0] * 0.2617993878:
    #             beta = fi - self.dir[0] * 0.2617993878  # I
    #         elif fi < shoulder_st[8] + self.dir[0] * 0.2617993878:
    #             beta = shoulder_st[8]  # II
    #         else:
    #             beta = shoulder_st[9]  # III
    #     else:  # inner
    #         alpha = knee_st[9]
    #         if shoulder_st[8] + self.dir[0] * 1.047197552 <= fi <= \
    #                 shoulder_st[9] + self.dir[0] * 1.047197552:
    #             beta = fi - self.dir[0] * 1.047197552  # IV
    #         elif fi <= shoulder_st[8] + self.dir[0] * 1.047197552:
    #             beta = shoulder_st[8]  # V
    #         else:
    #             beta = shoulder_st[9]  # VI
    #     return [alpha, beta, gama]

    def update(self, q):
        angles = self.get_angles()
        damp_state = p.getJointState(q.model, self.dampener)

        p.setJointMotorControl2(q.model, self.base, p.POSITION_CONTROL, angles[2],
                                force=p.getJointInfo(q.model, self.base)[10] / 100,
                                maxVelocity=p.getJointInfo(q.model, self.base)[11])
        p.setJointMotorControl2(q.model, self.shoulder, p.POSITION_CONTROL, angles[1],
                                force=p.getJointInfo(q.model, self.shoulder)[10] / 100,
                                maxVelocity=p.getJointInfo(q.model, self.shoulder)[11])
        p.setJointMotorControl2(q.model, self.knee, p.POSITION_CONTROL, angles[0],
                                force=p.getJointInfo(q.model, self.knee)[10] / 100,
                                maxVelocity=p.getJointInfo(q.model, self.knee)[11])
        p.setJointMotorControl2(q.model, self.heel, p.POSITION_CONTROL, -angles[0],
                                force=p.getJointInfo(q.model, self.heel)[10] / 100,
                                maxVelocity=p.getJointInfo(q.model, self.heel)[11])
        p.setJointMotorControl2(q.model, self.dampener, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.0,
                                force=self.stiffness_c * (damp_state[0] / self.damp_len))


    def make_plan(self, dest: DestPoint, state: FSMState):
        self.fsm.reset()
        self.plan.reset()

        self.plan.target = dest

        self.fsm.set(state)
        self.fsm.execute()

        return
