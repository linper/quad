import math
from tkinter import *
import numpy as np
from leg import Leg
from quad import Quad
from consts import *
from interp import *
import functools
from enum import IntEnum
import pybullet as p
import multiprocessing as mp
import time


class SPoint:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color


class GrView:
    def __init__(self, q_from: mp.Queue, q_to: mp.Queue):
        self.q_from: mp.Queue = q_from
        self.q_to: mp.Queue = q_to
        self.root = Tk()
        self.mul = 800
        self.height = 800
        self.width = 600
        self.root.title = "ground view"
        self.space = Canvas(self.root, background="white", height=self.height, width=self.width)
        self.space.bind("<Button-1>", self.btn_clk)
        self.space.bind("<Double-Button-1>", self.btn_dbl_clk)
        self.space.grid(row=0, column=0)
        self.saved: list = []
        b = Button(self.root, text="Go", command=self.send_go_cmd)
        b.place(x=0, y=self.height - 20)

    def draw_leg_circle(self, leg, q: Quad):
        force = q.sens_info.touch_force[leg.idx]

        if force > 0:
            color = "green"
        else:
            color = "red"

        r = 6
        pt = self.mul * leg.position * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        self.space.create_oval(pt[1] - r, pt[0] - r, pt[1] + r, pt[0] + r, fill=color)

    def draw_circle(self, pos, color):
        r = 6
        self.space.create_oval(pos[1] - r, pos[0] - r, pos[1] + r, pos[0] + r, fill=color)

    def draw_circle2(self, sp: SPoint):
        r = 6
        self.space.create_oval(sp.x - r, sp.y - r, sp.x + r, sp.y + r, fill=sp.color)

    def draw_force(self, q: Quad):
        reduced = self.mul * q.sens_info.t_force_info.pos * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        lpt = np.array([self.height / 2, self.width / 2, 0])
        self.space.create_line(lpt[1], lpt[0], reduced[1], reduced[0], width=4, fill="black")
        self.space.pack()
        self.draw_circle(reduced, q.sens_info.t_force_info.color)

    def draw_perimeter(self, q: Quad):
        t_legs = [l for i, l in enumerate(q.legs_cw) if q.sens_info.touch_force[l.idx] > 0]

        for i in range(len(t_legs)):
            lpt = self.mul * t_legs[i - 1].position * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
            cpt = self.mul * t_legs[i].position * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])

            self.space.create_line(lpt[1], lpt[0], cpt[1], cpt[0], width=4, fill="green")
            self.space.pack()

        g_legs = [l for i, l in enumerate(q.legs_cw) if l.grounded]

        for i in range(len(g_legs)):
            lpt = self.mul * g_legs[i - 1].position * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
            cpt = self.mul * g_legs[i].position * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])

            self.space.create_line(lpt[1], lpt[0], cpt[1], cpt[0], width=1, fill="green")
            self.space.pack()

    def draw_s_sines(self, q: Quad):
        sc = self.mul * q.sens_info.s_center * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        scl = self.mul * q.sens_info.to_s_closest * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        f = self.mul * q.sens_info.t_force_info.pos * np.array([1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        self.space.create_line(f[1], f[0], scl[1], scl[0], width=4, fill="blue")
        self.space.create_line(f[1], f[0], sc[1], sc[0], width=4, fill="blue")
        self.space.pack()

    def draw_saved(self):
        for s in self.saved:
            self.draw_circle2(s)

    def send_go_cmd(self):
        while self.q_to.full():
            time.sleep(0.05)

        print("Go")
        self.q_to.put(self.saved)
        # s = SPoint(ev.x, ev.y, new_path_col)
        # self.saved.append(s)
        # self.draw_circle2(s)

    def btn_clk(self, ev):
        new_path_col = "blue"
        # print("clicked")
        s = SPoint(ev.x, ev.y, new_path_col)
        self.saved.append(s)
        self.draw_circle2(s)

    def btn_dbl_clk(self, ev):
        # print("dbl clicked")
        err = 6
        for i, s in enumerate(self.saved):
            if abs(s.x - ev.x) <= err and abs(s.y - ev.y) <= err:
                self.saved.pop(i)


    def update(self, q: Quad):
        self.root.update()

        self.space.delete("all")

        self.draw_circle(np.array([self.height / 2, self.width / 2]), "black")
        self.draw_perimeter(q)

        for l in q.legs:
            self.draw_leg_circle(l, q)

        self.draw_s_sines(q)
        self.draw_force(q)
        self.draw_saved()

    def start(self):
        q: Quad
        while True:
            if self.q_from.empty():
                time.sleep(0.001)
            else:
                while not self.q_from.empty():
                    q = self.q_from.get()
                self.update(q)


def tk_start(q_from, q_to):
    gv = GrView(q_from, q_to)
    gv.root.after(200, gv.start)
    gv.root.mainloop()
