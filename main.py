import pybullet as p
import time
import numpy as np
import math
from ground_view import tk_start, SPoint
import matplotlib.pyplot as plt

from consts import *
from misc import ActCmd
from leg import Leg
from quad import Quad
import multiprocessing as mp

log_dt_len = 3000
log_dt_idx = 0
log_data: np.ndarray = np.zeros((5, log_dt_len), dtype=np.float)

def plot_heigths():
    plt.figure()
    time_data = list(range(log_dt_len))
    for i, l in enumerate(q.legs):
        plt.plot(time_data, log_data[i, :])

    plt.plot(time_data, log_data[4, :], color="black")
    plt.grid("both")
    plt.show()

def get_commands():
    global gv
    global log_dt_idx

    arr: list
    got_target = False

    while not q_from_gv.empty():
        arr = q_from_gv.get()
        got_target = True

    while not q_cmd.empty():
        cmd = q_cmd.get()
        if cmd == ActCmd.CLEAR:
            # for l in q.legs:
            #     l.position = l.def_pos
            arr = [np.array([0.0, 0.0, 0.0])]
            got_target = True
        elif cmd == ActCmd.PLOT:
            plot_heigths()
            # log_data.clear()


    q.update_sensor_info()

    q.avg_leg_h = np.average(np.array([l.position[2] + l.plan.adj[2] for l in q.legs]))
    print(f"leg avg. h:{q.avg_leg_h}:{[l.position[2] + l.plan.adj[2] for l in q.legs]}")

    log_entry = [l.position[2] + l.plan.adj[2] for l in q.legs]
    log_entry.append(q.avg_leg_h)
    log_data[:, log_dt_idx] = np.array(log_entry)
    log_dt_idx = (log_dt_idx + 1) % log_dt_len

    if got_target:
        q.set_target(arr)


    for l in q.legs:
        # if got_target:
        l.fsm.execute()
        l.update(q)

    # while q_to_gv.full():
    #     print("queue to GV is full")
    #     time.sleep(0.001)

    if q_to_gv.empty():
        q_to_gv.put(q)

    # q_to_gv.put(q)

# start point

p.connect(p.GUI)
plane = p.createCollisionShape(p.GEOM_PLANE)

p.createMultiBody(0, plane)
p.setGravity(0, 0, -10)
useMaximalCoordinates = False

# model = p.loadURDF("quad3.xacro", [0.0, 0.0, 0.5], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)
model = p.loadURDF("quad3.xacro", [0.0, 0.0, 0.5], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)

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
fll = Leg("front_left", base_j, shoulder_j, knee_j, heel_j, damp_j, sensor_j, BACK_DIR_LEFT, [0.0, -0.05, -0.23], off)

base_j = jointNameToId.get('back_left_base_to_shoulder')
shoulder_j = jointNameToId.get('back_left_shoulder')
knee_j = jointNameToId.get('back_left_knee')
heel_j = jointNameToId.get('back_left_heel')
damp_j = jointNameToId.get('back_left_damp_joint')
sensor_j = jointNameToId.get('back_left_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
bll = Leg("back_left", base_j, shoulder_j, knee_j, heel_j, damp_j, sensor_j, BACK_DIR_LEFT, [0.0, -0.05, -0.23], off)

base_j = jointNameToId.get('front_right_base_to_shoulder')
shoulder_j = jointNameToId.get('front_right_shoulder')
knee_j = jointNameToId.get('front_right_knee')
heel_j = jointNameToId.get('front_right_heel')
damp_j = jointNameToId.get('front_right_damp_joint')
sensor_j = jointNameToId.get('front_right_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
frl = Leg("front_right", base_j, shoulder_j, knee_j, heel_j, damp_j, sensor_j, BACK_DIR_RIGHT, [0.0, 0.05, -0.23], off)

base_j = jointNameToId.get('back_right_base_to_shoulder')
shoulder_j = jointNameToId.get('back_right_shoulder')
knee_j = jointNameToId.get('back_right_knee')
heel_j = jointNameToId.get('back_right_heel')
damp_j = jointNameToId.get('back_right_damp_joint')
sensor_j = jointNameToId.get('back_right_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, 1, -1])
brl = Leg("back_right", base_j, shoulder_j, knee_j, heel_j, damp_j, sensor_j, BACK_DIR_RIGHT, [0.0, 0.05, -0.23], off)

sensor_b = jointNameToId.get('base_sensor')  # we can use this as accelerometer and gyroscope
q: Quad = Quad(model, fll, frl, bll, brl, sensor_b)


base_orientation = np.array(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(q.model)[1]))
base_position = np.array(p.getBasePositionAndOrientation(q.model)[0])
touch_force_max = np.array([0, 0, 0, 0])

start_stop_param = p.addUserDebugParameter("stop/go", 0, 0.001, 0.)

x_par = p.addUserDebugParameter("x", -0.3, 0.3, 0)
y_par = p.addUserDebugParameter("y", -0.2, 0.2, 0.0)
z_par = p.addUserDebugParameter("z", -0.25, 0.2, 0.0)


# increase grip
for s in q.sensors:
    p.changeDynamics(q.model, s, lateralFriction=2)
    p.enableJointForceTorqueSensor(q.model, s, 1)
    print(p.getDynamicsInfo(q.model, s))
    print(p.getJointInfo(q.model, s)[1])

q_to_gv = mp.Queue(1)
q_from_gv = mp.Queue(10)
q_cmd = mp.Queue(10)

gv = mp.Process(target=tk_start, args=(q_to_gv, q_from_gv, q_cmd))
gv.start()
# q.to_starting_position()  # sets robot to starting position
counter = 0

INTERV = 4.166666666666667
l_time = int(1000 * time.time())
p.setRealTimeSimulation(0)

while 1:
    c_time = int(1000 * time.time())
    if c_time >= l_time + INTERV:
        l_time = c_time
        get_commands()
        p.stepSimulation()
    else:
        time.sleep(0.001)

