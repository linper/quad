import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
import time
import threading

from datetime import timedelta
from timeloop import Timeloop

exitFlag = 0

class myThread (threading.Thread):
   def __init__(self, threadID, name, counter):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.counter = counter
   def run(self):
      print ("Starting " + self.name)
      print_time(self.name, self.counter, 5)
      print ("Exiting " + self.name)

def print_time(threadName, delay, counter):
   while counter:
      if exitFlag:
         threadName.exit()
      time.sleep(delay)
      print ("%s: %s" % (threadName, time.ctime(time.time())))
      counter -= 1

# Create new threads
# thread1 = myThread(1, "Thread-1", 0.25)
# thread2 = myThread(2, "Thread-2", 0.5)

# Start new Threads
# thread1.start()
# thread2.start()
# thread1.join()
# thread2.join()
# print ("Exiting Main Thread")

import math
# tl = Timeloop()
interval = 0.004
# interval = 0.00417
freq = 1 / interval
old_time = time.perf_counter_ns()
new_time = time.perf_counter_ns()
old_time2 = time.perf_counter_ns()
new_time2 = time.perf_counter_ns()
inter_time = 0
counter = 0


# @tl.job(interval=timedelta(seconds=interval))
def timeManger2():
    global old_time
    global new_time
    global inter_time
    inter_time = time.perf_counter_ns()
    print((new_time - old_time) / 1000000)
    old_time = new_time
    new_time = inter_time
# tl.start(block=True)

def timeManger1():
    old = time.perf_counter_ns()
    print("*", (new_time - old_time) / 1000000)
    new = time.perf_counter_ns()
    print((new - old) / 1000000)
# tl.start(block=True)

def event_func():
    global old_time2
    global new_time2
    inter_time2 = time.perf_counter_ns()
    print("*", (new_time2 - old_time2) / 1000000)
    old_time2 = new_time2
    new_time2 = inter_time2

while True:
    old = time.perf_counter_ns()
    # print((new_time - old_time) / 1000000)
    # old_time = new_time
    # new_time = time.perf_counter_ns()
    if counter % 5 == 0:
        t2 = threading.Thread(target=event_func)
        t2.start()
    t = threading.Thread(target=timeManger2)
    # t = threading.Thread(target=timeManger1)
    t.start()
    new = time.perf_counter_ns()
    counter += 1
    time.sleep(abs(interval - (new - old)/1000000000))




#
# # Data
# # a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# # df = pd.DataFrame({'x': range(10), 'y1': np.random.randn(10), 'y2': np.random.randn(10) + range(1, 11),
# #                    'y3': np.random.randn(10) + range(11, 21)})
# #
# # # multiple line plot
# # plt.plot('x', 'y1', data=df, marker='o', markerfacecolor='blue', markersize=12, color='skyblue', linewidth=4)
# # plt.plot('y2', data=df, marker='', color='olive', linewidth=2)
# # plt.plot('y3', data=df, marker='', color='olive', linewidth=2, linestyle='dashed', label="toto")
# # plt.legend()
# # plt.grid()
# # plt.show()
# # a = np.array([0])
# #
# #
# # A = np.array([
# #     [[0, 0], [0, 1], [0, 2]],
# #     [[1, 0], [4, 1], [1, 2]],
# #     [[2, 0], [5, 1], [2, 2]],
# #     [[3, 0], [6, 1], [3, 2]]
# # ])
# # for i in np.nditer(A, op_flags = ['readwrite']):
# #     print(i)
# #     i[...] = 0
# # print(A)
# # print("END")
# # B = np.array([
# #     [0, 0, 2],
# #     [0, 1, 2]
# # ])
#
# # print(B.transpose())
# # print()
# # print(A[1:3, 1, 0])
# # print()
# # print(A.take([1][0], 1).transpose())
# # print()
# # print(A.take([1][0], 1).take([0, 1], 0))
# # print()
# # print(A.take([1][0], 1).take([0, 1], 0).compress([True, False], 1).flatten())
#
# a = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
# a = a + 2
# print(a)
# # print(np.delete(a, 2))
#
#
# forward = np.array([1, 0, 0])
# side = np.array([0, 1, 0])
# up = np.array([0, 0, 1])
#
# # print(np.copysign(1, -2))
#
# first = np.array([[1, 2, 3], [2, 3, 1], [3, 2, 1]])
# second = np.array([[1, 4, 7], [4, 7, 1], [7, 4, 1]])
#
# old_position = np.array([0, 0, 0])
# old_velocity = np.array([0, 0, 0])
# position = np.array([1, 0, 0])
#
# def ref(arr):
#     arr[1] = 0
#     return
# array = np.array([1, 2, 3])
# # print(array)
# ref(array)
# # print(array)
#
# def getAccelerationInfo(new_position):
#     global old_position
#     global old_velocity
#     gravity = np.array([0, 0, -9.81])
#     # new_velocity = 20 * (new_position - old_position)
#     new_velocity = 1 * (new_position - old_position)
#     old_position = new_position.copy()
#     # accel = gravity + new_velocity - old_velocity
#     accel = new_velocity - old_velocity
#     old_velocity = new_velocity.copy()
#     # print(accel)
#     return accel
#
# # getAccelerationInfo(position)
# # position[0] = 3
# # getAccelerationInfo(position)
# # position[0] = 6
# # getAccelerationInfo(position)
#
#
#
#
# def inertialize3DData(data_container, newdata, coafitient):
#     newdata = newdata.tolist()
#     if len(data_container) < coafitient:
#         data_container.append(newdata)
#     else:
#         data_container.pop(0)
#         data_container.append(newdata)
#     ax = 0
#     ay = 0
#     az = 0
#     for i in range(len(data_container)):
#         ax += data_container[i][0]
#         ay += data_container[i][1]
#         az += data_container[i][2]
#     return np.array([ax, ay, az]) / len(data_container)
#
#
# def getCrossProduct(a, b):
#     # return np.cross(np.array(a), np.array(b))
#     return np.array([a[1] * b[2] - a[2] * b[1],  a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])
#
#
# def getVectorsCosine(a, b):
#     return (np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b))
#
# def getPlainNormal(a, b, c):
#     a = np.array(a)
#     b = np.array(b)
#     c = np.array(c)
#     ab = b - a
#     ac = c - a
#
#
# def getRotationMatrixFromTwoVectors(a, b):
#     # if a.sum() == 0 or b.sum() == 0:
#     #     return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
#     a = streachVectorTo(a, 1)
#     b = streachVectorTo(b, 1)
#     phi = math.acos(getVectorsCosine(a, b))
#     axis = streachVectorTo(getCrossProduct(a, b), 1)
#     matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
#     rcos = math.cos(phi)
#     rsin = math.sin(phi)
#     t = 1 - rcos
#     matrix[0][0] = rcos + axis[0] * axis[0] * t
#     matrix[1][0] = axis[2] * rsin + axis[1] * axis[0] * t
#     matrix[2][0] = -axis[1] * rsin + axis[2] * axis[0] * t
#     matrix[0][1] = -axis[2] * rsin + axis[0] * axis[1] * t
#     matrix[1][1] = rcos + axis[1] * axis[1] * t
#     matrix[2][1] = axis[0] * rsin + axis[2] * axis[1] * t
#     matrix[0][2] = axis[1] * rsin + axis[0] * axis[2] * t
#     matrix[1][2] = -axis[0] * rsin + axis[1] * axis[2] * t
#     matrix[2][2] = rcos + axis[2] * axis[2] * t
#     # print(matrix)
#     return matrix
#
#
# def streachVectorTo(v, lenght):
#     l = np.linalg.norm(v)
#     ratio = lenght / l
#     return v * ratio
#
#
#
#
