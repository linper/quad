from tkinter import *
import numpy as np
from leg import Leg
from quad import Quad
from consts import *


class GrView:
    def __init__(self, q):
        self.q = q
        self.root = Tk()
        self.mul = 800
        self.height = 800
        self.width = 600
        self.root.title = "ground view"
        self.space = Canvas(self.root, background="white", height=self.height, width=self.width)
        self.space.pack()
        # b = Button(root, text="show all/best", command=change_state)
        # b.place(x=0, y=height - 20)
        self.space.grid(row=0, column=0)
        # self.params = Text(self.root, background="grey", height=6, width=150)
        # self.params.grid(row=1, column=0)
        # self.params.pack()

    def draw_leg_circle(self, leg):
        force = self.q.sens_info.touch_force[leg.idx]

        if force > 0:
            color = "green"
        else:
            color = "red"

        r = 6
        pt = self.mul * leg.position * np.array([-1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        self.space.create_oval(pt[1] - r, pt[0] - r, pt[1] + r, pt[0] + r, fill=color)

    def draw_circle(self, pos, color):
        r = 6
        self.space.create_oval(pos[1] - r, pos[0] - r, pos[1] + r, pos[0] + r, fill=color)

    def draw_force(self):
        force = np.copy(self.q.sens_info.base_force_vector)
        force = force / np.linalg.norm(force)
        r = 6
        pt = 0.1 * self.mul * force * np.array([-1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        self.space.create_oval(pt[1] - r, pt[0] - r, pt[1] + r, pt[0] + r, fill="blue")

    def draw_perimeter(self):
        found_first = False
        last: Leg
        for i in range(-1, len(self.q.legs)):
            cur = self.q.legs[clock_wise_sequence[i]]
            f = self.q.sens_info.touch_force[cur.idx]
            if f == 0:
                continue

            if not found_first:
                last = cur
                found_first = True
                continue

            lpt = self.mul * last.position * np.array([-1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
            cpt = self.mul * cur.position * np.array([-1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])

            self.space.create_line(lpt[1], lpt[0], cpt[1], cpt[0], width=4, fill="green")
            self.space.pack()
            last = cur

    def update(self):
        self.root.update()

        self.space.delete("all")

        self.draw_circle(np.array([self.height / 2, self.width / 2]), "black")
        self.draw_perimeter()

        for l in self.q.legs:
            self.draw_leg_circle(l)

        self.draw_force()



