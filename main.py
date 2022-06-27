import pybullet as p
import time
import numpy as np
import math
from ground_view import tk_start, SPoint
import threading

from consts import *
from leg import Leg
from quad import Quad
import multiprocessing as mp

def get_commands():
    global gv

    arr: list
    got_target = False

    while not q_from_gv.empty():
        arr = q_from_gv.get()
        got_target = True

    q.update_sensor_info()

    # if got_target:
    #     q.set_target(arr)
    #
    # cor = q.set_step()

    st_st_val = p.readUserDebugParameter(start_stop_param)
    cor = np.array([p.readUserDebugParameter(x_par),
         p.readUserDebugParameter(y_par),
         p.readUserDebugParameter(z_par)])



    for l in q.legs:
        if round(st_st_val, 3) == 0:
            l.position = l.def_pos + l.base_off
        else:
            l.position = cor + l.def_pos + l.base_off
        l.update(q)

    while q_to_gv.full():
        print("queue to GV is full")
        time.sleep(0.001)

    q_to_gv.put(q)



# start point

p.connect(p.GUI)
plane = p.createCollisionShape(p.GEOM_PLANE)

p.createMultiBody(0, plane)
p.setGravity(0, 0, -10)
useMaximalCoordinates = False

# model = p.loadURDF("quad3.xacro", [0.0, 0.0, 0.5], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)
model = p.loadURDF("quad3.xacro", [0.0, 0.0, 0.5], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
# model = p.loadURDF("quad2.xacro", [0.0, 0.0, 0.2], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
# model = p.loadURDF("quad2.xacro", [0.0, 0.0, 0.2], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)
# model = p.loadURDF("quad.xacro", [0.0, 0.0, 0.2], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
# model = p.loadURDF("quad.xacro", [0.0, 0.0, 0.2], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)

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

# setting manual parameters for debugging
# speed = p.addUserDebugParameter("speed", -0.15, 0.3, 0)
# step = p.addUserDebugParameter("step", -0.1, 0.1, 0)
# cross_step = p.addUserDebugParameter("cross step", -0.1, 0.1, 0)
# turn = p.addUserDebugParameter("turn", -0.5235988, 0.5235988, 0)
# side_lean = p.addUserDebugParameter("side lean", -0.5, 0.5, 0.0)
# front_lean = p.addUserDebugParameter("front lean", -math.pi / 9, math.pi / 9, 0)
# inclination = p.addUserDebugParameter("inclination", -0.8, 0.8, 0.0)
# sit_up = p.addUserDebugParameter("sit up", -0.04, 0.04, 0)
# step_height = p.addUserDebugParameter("step height", 0, 0.3, 0)
# spread = p.addUserDebugParameter("spread", 0, math.pi / 6, math.pi / 60)

start_stop_param = p.addUserDebugParameter("stop/go", 0, 0.001, 0.)

# alpha_par = p.addUserDebugParameter("alpha", -2.617993877991494, -0.52359877559829884, -1.5707)
# beta_par = p.addUserDebugParameter("beta", -math.pi/2, math.pi/2, 0)
# gama_par = p.addUserDebugParameter("gama", -math.pi/2, math.pi/2, 0)
x_par = p.addUserDebugParameter("x", -0.3, 0.3, 0)
y_par = p.addUserDebugParameter("y", -0.2, 0.2, 0.0)
z_par = p.addUserDebugParameter("z", -0.25, 0.2, 0.0)


# increase grip
for s in q.sensors:
    p.changeDynamics(q.model, s, lateralFriction=2)
    p.enableJointForceTorqueSensor(q.model, s, 1)
    print(p.getDynamicsInfo(q.model, s))
    print(p.getJointInfo(q.model, s)[1])

q_to_gv = mp.Queue()
q_from_gv = mp.Queue()

gv = mp.Process(target=tk_start, args=(q_to_gv, q_from_gv))
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

# to_starting_position()  # sets robot to starting position
# counter = 0


# while 1:
#     p.stepSimulation()
#     # high_freq()  # measuring pressure sensors more often
#     if counter % 5 == 0:
#         t = threading.Thread(target=get_commands)  # base frequency is 50 Hz
#         t.start()
#     counter += 1
#
#     time.sleep(1 / 250)

