import pybullet as p
import json
import time
import numpy as np
import time
import os
from enum import IntEnum

from data_models import Quad, Leg
from consts import T_RAD

SIM_CTL_PIPE = "/tmp/sim_ctl_pipe"
CTL_SIM_PIPE = "/tmp/ctl_sim_pipe"

sc_pp = None
cs_pp = None

q = None
# signed directions
FRONT_DIR_LEFT = [1, -1, -1]
FRONT_DIR_RIGHT = [1, -1, 1]
BACK_DIR_LEFT = [-1, 1, -1]
BACK_DIR_RIGHT = [-1, 1, 1]
LEFT_POS = np.array([0.0, -0.05, -0.23])
RIGHT_POS = np.array([0.0, 0.05, -0.23])


class STATUS(IntEnum):
    OK = 0
    ERROR = 1
    NOT_INIT = 2
    NO_DATA = 3
    PARSE = 4
    SUPPORT = 5
    MAX__ = 6


def update_joints(data):
    for i, l in enumerate(q.legs):
        angles = data[i]
        damp = q.sens_info.damp[i] * T_RAD

        p.setJointMotorControl2(q.model, l.base, p.POSITION_CONTROL, angles[2],
                                force=p.getJointInfo(q.model, l.base)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, l.base)[11])
        p.setJointMotorControl2(q.model, l.shoulder, p.POSITION_CONTROL, angles[1],
                                force=p.getJointInfo(q.model, l.shoulder)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, l.shoulder)[11])
        p.setJointMotorControl2(q.model, l.knee, p.POSITION_CONTROL, angles[0],
                                force=p.getJointInfo(q.model, l.knee)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, l.knee)[11])
        p.setJointMotorControl2(q.model, l.heel, p.POSITION_CONTROL, -angles[0],
                                force=p.getJointInfo(q.model, l.heel)[
            10] / 100,
            maxVelocity=p.getJointInfo(q.model, l.heel)[11])
        p.setJointMotorControl2(q.model, l.dampener, controlMode=p.POSITION_CONTROL,
                                targetPosition=0.0,
                                force=l.stiffness_c * (damp / l.damp_len))


def connect_pe():
    p.connect(p.GUI)
    # p.setTimeStep(1/50)
    p.setRealTimeSimulation(0)


def load_env():
    obj1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.5, 0.04], collisionFramePosition=[
                                  0, 0.8, 0.025], collisionFrameOrientation=[0.08, 0, 0, 1])
    obj2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.5, 0.04], collisionFramePosition=[
                                  0, 0.75, -0.015], collisionFrameOrientation=[0, 0, 0, 1])
    obj3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[
                                  0.02, 0.5, 0.04], collisionFramePosition=[-1, 0, -0.02], collisionFrameOrientation=[0, 0, 0, 1])
    obj4 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[
                                  0.02, 0.5, 0.04], collisionFramePosition=[-1.1, 0, -0.015], collisionFrameOrientation=[0, 0, 0, 1])
    obj5 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[
                                  0.02, 0.5, 0.04], collisionFramePosition=[-1.18, 0, -0.02], collisionFrameOrientation=[0, 0, 0, 1])
    obj6 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[
                                  0.02, 0.5, 0.04], collisionFramePosition=[-1.23, 0, -0.025], collisionFrameOrientation=[0, 0, 0, 1])
    obj7 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[
                                  0.02, 0.5, 0.04], collisionFramePosition=[-1.3, 0, -0.02], collisionFrameOrientation=[0, 0, 0, 1])
    obj8 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[
                                  0.02, 0.5, 0.04], collisionFramePosition=[-1.4, 0, -0.015], collisionFrameOrientation=[0, 0, 0, 1])

    p.createMultiBody(0, obj1)
    p.createMultiBody(0, obj2)
    p.createMultiBody(0, obj3)
    p.createMultiBody(0, obj4)
    p.createMultiBody(0, obj5)
    p.createMultiBody(0, obj6)
    p.createMultiBody(0, obj7)
    p.createMultiBody(0, obj8)

    plane = p.createCollisionShape(p.GEOM_PLANE)

    p.createMultiBody(0, plane)
    p.setGravity(0, 0, -10)


def load_robot():
    useMaximalCoordinates = False
    # model = p.loadURDF("quad3.xacro", [0.0, 0.0, 0.5], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)
    model = p.loadURDF("resources/quad.xacro", [
                       0.0, 0.0, 0.5], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
    return model


def build_model(model):
    # Getting joints indices
    nJoints = p.getNumJoints(model)
    jointNameToId = {}
    for k in range(nJoints):
        jointInfo = p.getJointInfo(model, k)
        jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

    base_j = jointNameToId.get('front_left_base_to_shoulder')
    shoulder_j = jointNameToId.get('front_left_shoulder')
    knee_j = jointNameToId.get('front_left_knee')
    heel_j = jointNameToId.get('front_left_heel')
    damp_j = jointNameToId.get('front_left_damp_joint')
    sensor_j = jointNameToId.get('front_left_sensor')
    off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
    fll = Leg("front_left", base_j, shoulder_j, knee_j, heel_j, damp_j,
              sensor_j, BACK_DIR_LEFT, LEFT_POS, off)

    base_j = jointNameToId.get('back_left_base_to_shoulder')
    shoulder_j = jointNameToId.get('back_left_shoulder')
    knee_j = jointNameToId.get('back_left_knee')
    heel_j = jointNameToId.get('back_left_heel')
    damp_j = jointNameToId.get('back_left_damp_joint')
    sensor_j = jointNameToId.get('back_left_sensor')
    off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
    bll = Leg("back_left", base_j, shoulder_j, knee_j, heel_j, damp_j,
              sensor_j, BACK_DIR_LEFT, LEFT_POS, off)

    base_j = jointNameToId.get('front_right_base_to_shoulder')
    shoulder_j = jointNameToId.get('front_right_shoulder')
    knee_j = jointNameToId.get('front_right_knee')
    heel_j = jointNameToId.get('front_right_heel')
    damp_j = jointNameToId.get('front_right_damp_joint')
    sensor_j = jointNameToId.get('front_right_sensor')
    off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
    frl = Leg("front_right", base_j, shoulder_j, knee_j, heel_j,
              damp_j, sensor_j, BACK_DIR_RIGHT, RIGHT_POS, off)

    base_j = jointNameToId.get('back_right_base_to_shoulder')
    shoulder_j = jointNameToId.get('back_right_shoulder')
    knee_j = jointNameToId.get('back_right_knee')
    heel_j = jointNameToId.get('back_right_heel')
    damp_j = jointNameToId.get('back_right_damp_joint')
    sensor_j = jointNameToId.get('back_right_sensor')
    off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
    brl = Leg("back_right", base_j, shoulder_j, knee_j, heel_j, damp_j,
              sensor_j, BACK_DIR_RIGHT, RIGHT_POS, off)

    # we can use this as accelerometer and gyroscope
    sensor_b = jointNameToId.get('base_sensor')
    q: Quad = Quad(model, fll, frl, bll, brl, sensor_b)

    # increase grip
    for s in q.sensors:
        p.changeDynamics(q.model, s, lateralFriction=2)
        p.enableJointForceTorqueSensor(q.model, s, 1)
        print(p.getDynamicsInfo(q.model, s))
        print(p.getJointInfo(q.model, s)[1])

    return q

#########
#  API  #
#########


RES_OK = {"status":  STATUS.OK}
RES_ERROR = {"status":  STATUS.ERROR}
NO_MODEL = {"status": STATUS.NOT_INIT}
NO_DATA = {"status": STATUS.NO_DATA}
PARSE_FAIL = {"status": STATUS.PARSE}
NOT_SUPPORTED = {"status": STATUS.SUPPORT}

RSP_EMPTY = {"rsp": "empty"}


def api_setup(req, data):
    global q

    if q is not None:
        req.update(RES_OK)
        return

    connect_pe()
    load_env()
    m = load_robot()
    q = build_model(m)
    time.sleep(0.03)

    req.update(RES_OK)


def api_stop(req, data):
    os.write(sc_pp, bytes(json.dumps(RES_OK), "utf-8"))
    quit(0)


def api_step(req, data):
    if q is None:
        req.update(NO_MODEL)
        return

    parsed_array = data.get("angles")
    if (parsed_array is None or len(parsed_array) != 4):
        req.update(NO_DATA)
        return

    for _ in range(1):
        update_joints(parsed_array)
        p.stepSimulation()

    q.sens_info.update()
    req.update(RES_OK)
    req.update({"rsp": q.sens_info.get_json()})
    # time.sleep(0.02)
    # time.sleep(0.015)
    time.sleep(0.007)


def api_echo(req, data):
    if data is None:
        req.update(NO_DATA)
        return

    req.update(RES_OK)
    req.update({"rsp": data})


def api_model(req, data):
    if q is None:
        req.update(NO_MODEL)
        return

    req.update(RES_OK)
    req.update({"rsp": q.get_json()})


def api_sensors(req, data):
    if q is None:
        req.update(NO_MODEL)
        return

    req.update(RES_OK)
    req.update({"rsp": q.sens_info.get_json()})


def listen_ctl():
    global sc_pp, cs_pp

    try:
        os.mkfifo(SIM_CTL_PIPE)
    except:
        pass
    try:
        os.mkfifo(CTL_SIM_PIPE)
    except:
        pass

    sc_pp = os.open(SIM_CTL_PIPE, os.O_RDWR)

    while True:
        cs_pp = os.open(CTL_SIM_PIPE, os.O_RDONLY)
        while True:
            b_msg = os.read(cs_pp, 2048)
            if len(b_msg) == 0:  # Writter closed
                break

            msg = b_msg.decode("utf-8")
            print(f"Recieved {msg}")
            # print(f"HEX{':'.join('{: 02x}'.format(ord(c)) for c in msg)}")
            try:
                json_msg = json.loads(msg)
            except:
                print("Writting failed to parse message")
                os.write(sc_pp, bytes(PARSE_FAIL, "utf-8"))
                print("Written failed to parse message")
                continue

            data = json_msg.get("data")
            msg_id = json_msg.get("id")
            act_str = json_msg.get("act")
            act = actions.get(act_str)
            req = {"id": -1 if msg_id is None else msg_id,
                   "act": act_str, "rsp": "empty"}
            if act is not None:
                print(f"executing:{act_str}")
                act(req, data)
            else:
                req.update(NOT_SUPPORTED)

            os.write(sc_pp, bytes(json.dumps(req), "utf-8"))

        os.close(cs_pp)


actions = {
    "setup": api_setup,
    "exit": api_stop,
    "step": api_step,
    "model": api_model,
    "sens": api_sensors,
    "echo": api_echo,
}


if __name__ == "__main__":
    listen_ctl()

    # connect_pe()
    # load_env()
    # m = load_robot()
    # q = build_model(m)
    # while(True):
    # p.stepSimulation()
    # q.sens_info.update()
    # time.sleep(0.02)
