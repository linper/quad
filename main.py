import pybullet as p
import time
import numpy as np
import math
from ground_view import GrView
from threading import Thread

from consts import *
from leg import Leg
from quad import Quad


def get_commands():
    global last_movement_properties
    global touch_force_max
    global last_time
    global starting_speed
    global movement_command
    global gv

    movement_command = p.readUserDebugParameter(param2)
    q.update_sensor_info()
    gv.update()

    # reads debugging parameters as commands
    # last_movement_properties = np.array([
    #     [p.readUserDebugParameter(step),
    #      p.readUserDebugParameter(cross_step),
    #      p.readUserDebugParameter(sit_up),
    #      p.readUserDebugParameter(speed)],
    #     [p.readUserDebugParameter(side_lean),
    #      p.readUserDebugParameter(inclination),
    #      p.readUserDebugParameter(turn),
    #      p.readUserDebugParameter(front_lean)],
    #     [p.readUserDebugParameter(step_height),
    #      p.readUserDebugParameter(spread), 0, 0]])

    # ang = np.array([p.readUserDebugParameter(alpha_par),
    #      p.readUserDebugParameter(beta_par),
    #      p.readUserDebugParameter(gama_par)])
    cor = np.array([p.readUserDebugParameter(x_par),
         p.readUserDebugParameter(y_par),
         p.readUserDebugParameter(z_par)])

    for l in q.legs:
        l.position = cor
        l.update(q)

    # q.front_left_leg.update(q, cor)
    # q.back_left_leg.update(q, cor)
    # q.front_right_leg.update(q, cor)
    # q.back_right_leg.update(q, cor)

    touch_force_max = np.array([0, 0, 0, 0])



# start point

p.connect(p.GUI)
plane = p.createCollisionShape(p.GEOM_PLANE)
# ball = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)

# p.createMultiBody(0, ball, basePosition= [1., 0., 0.4])
p.createMultiBody(0, plane)
p.setGravity(0, 0, -10)
useMaximalCoordinates = False

# model = p.loadURDF("quad2.xacro", [0.0, 0.0, 0.2], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
# model = p.loadURDF("quad2.xacro", [0.0, 0.0, 0.2], useFixedBase=True, useMaximalCoordinates=useMaximalCoordinates)
model = p.loadURDF("quad.xacro", [0.0, 0.0, 0.2], useFixedBase=False, useMaximalCoordinates=useMaximalCoordinates)
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
sensor_j = jointNameToId.get('front_left_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
fll = Leg("front_left", base_j, shoulder_j, knee_j, heel_j, sensor_j, FRONT_DIR_LEFT, [0.125, -0.065, -0.24], off)

base_j = jointNameToId.get('back_left_base_to_shoulder')
shoulder_j = jointNameToId.get('back_left_shoulder')
knee_j = jointNameToId.get('back_left_knee')
heel_j = jointNameToId.get('back_left_heel')
sensor_j = jointNameToId.get('back_left_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
bll = Leg("back_left", base_j, shoulder_j, knee_j, heel_j, sensor_j, BACK_DIR_LEFT, [0.125, 0.065, -0.24], off)

base_j = jointNameToId.get('front_right_base_to_shoulder')
shoulder_j = jointNameToId.get('front_right_shoulder')
knee_j = jointNameToId.get('front_right_knee')
heel_j = jointNameToId.get('front_right_heel')
sensor_j = jointNameToId.get('front_right_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
frl = Leg("front_right", base_j, shoulder_j, knee_j, heel_j, sensor_j, FRONT_DIR_RIGHT, [-0.125, -0.065, -0.24], off)

base_j = jointNameToId.get('back_right_base_to_shoulder')
shoulder_j = jointNameToId.get('back_right_shoulder')
knee_j = jointNameToId.get('back_right_knee')
heel_j = jointNameToId.get('back_right_heel')
sensor_j = jointNameToId.get('back_right_sensor')
off = np.array(p.getJointInfo(model, base_j)[14]) * np.array([1, -1, -1])
brl = Leg("back_right", base_j, shoulder_j, knee_j, heel_j, sensor_j, BACK_DIR_RIGHT, [-0.125, 0.065, -0.24], off)

sensor_b = jointNameToId.get('base_sensor')  # we can use this as accelerometer and gyroscope
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
for s in q.sensors:
    p.changeDynamics(q.model, s, lateralFriction=2)
    p.enableJointForceTorqueSensor(q.model, s, 1)
    print(p.getDynamicsInfo(q.model, s))
    print(p.getJointInfo(q.model, s)[1])

gv = GrView(q)

# q.to_starting_position()  # sets robot to starting position
counter = 0

INTERV = 20
l_time = int(1000 * time.time())

while 1:
    c_time = int(1000 * time.time())
    if c_time >= l_time + INTERV:
        l_time = c_time
        # for i in range(4):
        #     touch_force_max[i] = max(-p.getJointState(q.model, q.sensors[i])[2][2], touch_force_max[i])
        get_commands()
        p.stepSimulation()
    else:
        time.sleep(0.005)

