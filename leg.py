import math

import pybullet as p
from fsm import FSM, FSMState
from plan import *
from plan import DestPoint
from pidc import PIDC


class LegView:
    def __init__(self, l):
        self.idx: int = l.idx
        self.name: str = l.name
        self.position = l.position
        self.def_pos = l.def_pos
        self.base_off = l.base_off
        self.plan = l.plan.get_view()
        self.fsm: FSM = l.fsm.get_view()
        self.grounded = l.grounded
        self.do_balance: bool = l.do_balance


class Leg:
    def __init__(self, name, base, shoulder, knee, heel, damp, sensor, dir, pos, off):
        ps = np.array(pos)
        bo = np.array(off)

        self.next: Leg = None
        self.prev: Leg = None
        self.idx: int = -1
        self.name: str = name
        self.body = None
        self.position = ps + bo
        self.def_pos = ps + bo
        self.base_off = bo
        self.plan = Plan(self)
        self.fsm: FSM = FSM(self)
        self.balance_pid: PIDC = PIDC(0.06, 0., 0.0005, 1 / 240)
        # self.balance_pid: PIDC = PIDC(0.06, 0., 0.001, 1 / 240)
        self.touch_pid: PIDC = PIDC(0.35, 0., 0., 1 / 240)
        self.base = base
        self.shoulder = shoulder
        self.knee = knee
        self.heel = heel
        self.dampener = damp
        self.sensor = sensor
        self.grounded = True
        self.dir = dir
        self.sh_w = 0.02
        self.sh_h = 0.03
        self.link_len = 0.1
        self.damp_len = 0.012
        self.stiffness_c = 0.00005
        # self.ellipse_a = 0.21
        # self.ellipse_b = 0.18
        # self.stiffness_c = 2.5
        self.do_balance: bool = True

    @classmethod
    def dummy(cls):
        obj = cls("dummy", None, None, None, None, None, None, np.zeros(
            3), np.zeros(3, dtype=float), np.zeros(3, dtype=float))
        return obj

    def get_view(self):
        return LegView(self)

    def get_angles(self) -> list:
        target = np.copy(self.position) - self.base_off
        # target = np.copy(self.position)

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
        max_leg_len = self.link_len * \
            math.sqrt(5 - 4 * math.cos(abs(knee_st[9])))
        min_leg_len = self.link_len * \
            math.sqrt(5 - 4 * math.cos(abs(knee_st[8])))

        if max_leg_len > leg_length > min_leg_len:  # middle
            e = math.acos(
                1.25 - ((leg_length ** 2) / (4 * self.link_len ** 2)))
            alpha = self.dir[0] * (math.pi - e)
        elif leg_length >= max_leg_len:  # outer
            alpha = knee_st[8]
        else:  # inner
            alpha = knee_st[9]

        fi = math.asin(target[0] / leg_length)
        theta = math.asin(
            min(1, max(-1, (self.link_len * math.sin(alpha)) / leg_length)))

        if shoulder_st[8] <= -fi + theta * -1 <= shoulder_st[9]:
            beta = -fi + theta * -1  # VII
        elif shoulder_st[8] > -fi + theta * -1:
            beta = shoulder_st[8]  # VIII
        else:
            beta = shoulder_st[9]

        return [alpha, beta, gama]

    def check_damp(self):
        l_idx = self.idx
        l_tf = self.body.sens_info.touch_force[l_idx]

        damp_val = p.getJointState(self.body.model, self.dampener)[0]

        damp_val_n = damp_val / T_RAD

        soft_hit = True if damp_val_n > SOFT_HIT_THR else False

        dst = 1.0 * (damp_val - T_RAD)
        return bool(l_tf > 0), soft_hit, dst

    def update(self, q):
        angles = self.get_angles()
        damp_state = p.getJointState(q.model, self.dampener)

        p.setJointMotorControl2(q.model, self.base, p.POSITION_CONTROL, angles[2],
                                force=p.getJointInfo(q.model, self.base)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, self.base)[11])
        p.setJointMotorControl2(q.model, self.shoulder, p.POSITION_CONTROL, angles[1],
                                force=p.getJointInfo(q.model, self.shoulder)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, self.shoulder)[11])
        p.setJointMotorControl2(q.model, self.knee, p.POSITION_CONTROL, angles[0],
                                force=p.getJointInfo(q.model, self.knee)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, self.knee)[11])
        p.setJointMotorControl2(q.model, self.heel, p.POSITION_CONTROL, -angles[0],
                                force=p.getJointInfo(q.model, self.heel)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, self.heel)[11])
        p.setJointMotorControl2(q.model, self.dampener, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.0,
                                # force=self.stiffness_c * (1-(damp_state[0] / self.damp_len)))
                                force=self.stiffness_c * (damp_state[0] / self.damp_len))

    def make_plan(self, pts: list, state: FSMState, speed_func=do_nothing):
        self.fsm.reset()
        self.plan.reset()
        self.plan.speed_func = speed_func
        self.plan.raw_points = pts
        self.plan.target = pts[-1]

        self.fsm.set(state)

        return

    def make_plan2(self, steps: list, state: FSMState):
        for s in steps:
            s.pos = s.pos + self.position

        p = self.plan
        fsm = self.fsm

        fsm.reset()
        p.reset()
        p.steps = steps
        p.need_plan = False
        p.target = steps[-1]

        fsm.set(state)

        return
