from tkinter import *
import numpy as np
from leg import Leg
from quad import Quad

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

    def draw_leg_circle(self, leg, color):
        r = 6
        pt = self.mul * leg.position * np.array([-1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        # pt = self.mul * (leg.position + leg.base_off) * np.array([-1, 1, 1]) + np.array([self.height / 2, self.width / 2, 0])
        self.space.create_oval(pt[1] - r, pt[0] - r, pt[1] + r, pt[0] + r, fill=color)


    def update(self):
        self.root.update()

        self.space.delete("all")

        for l in self.q.legs:
            self.draw_leg_circle(l, "green")
            # self.space.create_oval(mid_x -10, mid_y -10, mid_x +10, mid_y +10)
            # self.space.create_oval(mid_x + 100 * l.position[0], mid_y + 100 * l.position[1])



