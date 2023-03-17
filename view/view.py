import math
import tkinter as tk
import numpy as np
import os
# from leg import Leg
# from quad import Quad
# from consts import *
# from fsm import FSM
# from interp import *
import json
import functools
from enum import IntEnum
import time
# from misc import ActCmd

VC_PATH = "/tmp/view_ctl_pipe"
CV_PATH = "/tmp/ctl_view_pipe"

CW = [0, 1, 3, 2]
SOFT_HIT_THR = 0.02

tf_col_tbl = {
    3: "green",
    2: "yellow",
    1: "orange",
    0: "red",
}


class VSens():
    def __init__(self, touch_force, damp, tf_pos, tf_type):
        # print(f"SENS:{touch_force}:{damp}:{tf_pos}:{tf_type}")
        # self.avg_leg_h: float = si.avg_leg_h
        # self.abs_std_leg_h: float = si.abs_std_leg_h
        self.touch_force: np.ndarray = np.array(touch_force)
        self.damp: np.ndarray = np.array(damp)
        self.tf_pos: np.ndarray = np.array(tf_pos)
        self.tf_col: str = tf_col_tbl.get(tf_type)
        # self.s_center = si.s_center.copy()
        # self.to_s_closest = si.to_s_closest.copy()


class VLeg():
    def __init__(self, bal, idx, name, pos, def_pos):
        # print(f"LEG:{name}:{idx}:{pos}:{def_pos}")
        self.do_balance: bool = bool(bal)
        self.name: str = name
        self.idx: int = idx
        self.position: np.ndarray = np.array(pos)
        self.def_pos: np.ndarray = np.array(def_pos)
        # self.grounded = l.grounded


class VQuad():
    def __init__(self, legs, sens):
        self.sens_info: VSens = sens
        self.legs: list = legs
        self.legs_cw: list = [legs[CW[i]] for i in range(4)]



def line_cof(x1, y1, x2, y2):
    fi = 0.0000001

    k = (y2 - y1) / (x2 - x1 + fi)
    b = y1 - k * x1
    k = k if k != 0 else fi
    return k, b


def intersect(k1, b1, k2, b2):
    if round(k1, 4) == round(k2, 4):
        return False, 0, 0

    x = (b2 - b1) / (k1 - k2)
    y = k1 * x + b1
    return True, x, y


class GoPoint:
    def __init__(self, pos: np.ndarray):
        self.pos: np.ndarray = pos

    def clone(self):
        return GoPoint(self.pos.copy())


class GoTask:
    def __init__(self, idx: int, do_lift: bool = False):
        self.points: list=[]
        self.direction: np.ndarray=np.array([-0.1, 0.0])
        self.idx: int=idx
        self.do_lift: bool=do_lift
        self.active: bool=False
        self.exp_set: bool=False  # i.e. explicitly set
        self.dummy: bool=False

    def add_pt(self, pt: GoPoint):
        self.points.append(pt)

    def del_pt(self, idx: int):
        self.points.pop(idx)

    def toggle_lift(self):
        self.do_lift=not self.do_lift

    def clear(self):
        self.points.clear()
        self.do_lift=False
        self.active=False
        self.dummy=False
        self.exp_set=False

class GoCmd:
    def __init__(self, tasks):
        self.tasks: list=tasks
        self.test_mode=False


class SPoint:
    def __init__(self, x, y, color):
        self.x=x
        self.y=y
        self.color=color


class GrView:
    def __init__(self):
        self.root=tk.Tk()
        self.mul=800
        self.height=800
        self.width=600
        self.cntr=np.array([self.height / 2, self.width / 2, 0])
        self.root.title="View"
        # self.path_sent=False
        self.q=None
        self.sel_mode=tk.IntVar()
        self.test_mode=tk.IntVar()

        self.space=tk.Canvas(self.root, background = "white",
                            height = self.height, width = self.width)
        self.space.bind("<Button-1>", self.on_btn_clk)
        self.space.bind("<Double-Button-1>", self.on_btn_dbl_clk)
        self.space.grid(row = 0, column = 0)
        # self.saved: list = []
        self.tasks: list=[GoTask(-1), GoTask(0),
                                 GoTask(1), GoTask(2), GoTask(3)]
        self.task_cmd = GoCmd(self.tasks)
        b= tk.Button(self.root, text = "Go", command =self.send_go_cmd)
        b.place(x = 0, y =self.height - 20)
        b= tk.Button(self.root, text = "Clear", command =self.send_clear_cmd)
        b.place(x = 0, y =self.height - 50)
        b= tk.Button(self.root, text = "Start/Stop", command =self.on_plot_btn_clk)
        b.place(x = 0, y =self.height - 80)
        R1= tk.Radiobutton(self.root, text = "Set target", variable =self.sel_mode, value=0)
        R1.place(x = 0, y =self.height + 25)

        R2= tk.Radiobutton(self.root, text = "Set direction", variable =self.sel_mode, value=1)
        R2.place(x = 0, y =self.height + 5)
        self.sel_mode.set(0)

        R3= tk.Radiobutton(self.root, text = "Do move", variable=self.test_mode, value=0, command=self.sel_test_mode)
        R3.place(x = 100, y =self.height + 25)

        R4= tk.Radiobutton(self.root, text = "Test move", variable=self.test_mode, value=1, command=self.sel_test_mode)
        R4.place(x = 100, y =self.height + 5)
        self.test_mode.set(0)
        self.running: bool = False

        try:
            os.mkfifo(VC_PATH)
        except:
            pass
        try:
            os.mkfifo(CV_PATH)
        except:
            pass

        self.vc_fd = os.open(VC_PATH, os.O_WRONLY)
        self.cv_fd = os.open(CV_PATH, os.O_RDWR)

        self.root.tk.createfilehandler(
            self.cv_fd, tk.READABLE, self.on_got_msg)

    def sel_test_mode(self):
        self.task_cmd.test_mode= bool(self.test_mode.get())

    def draw_legs_middle(self):
        r= 5

        pos_lst= [l.position for l in self.q.legs if l.do_balance]

        if len(pos_lst) == 0:
            return

        pos= np.average(np.array(pos_lst), axis = 0)

        pt = self.mul * pos + self.cntr

        self.space.create_oval(pt[1] - r, pt[0] - r,
                               pt[1] + r, pt[0] + r, fill = "black")
        # self.space.create_oval(pt[0] - r, pt[1] - r,
                               # pt[0] + r, pt[1] + r, fill="black")


    def draw_leg_circle(self, leg):
        force=self.q.sens_info.touch_force[leg.idx]
        damp_val_n=self.q.sens_info.damp[leg.idx]

        if force > 0:
            color="green"
        elif damp_val_n > SOFT_HIT_THR:
            color="yellow"
        else:
            color="red"

        r=6
        pt=self.mul * leg.position + \
            self.cntr
        self.space.create_oval(pt[1] - r, pt[0] - r,
                               pt[1] + r, pt[0] + r, fill = color)
        leg_name_abr="".join([c[0] for c in leg.name.split("_")])
        self.space.create_text(pt[1] - r - 40, pt[0] - r - 10,
                               anchor = tk.W, text =f"{leg_name_abr}:{round(leg.position[2], 3)}")
        # self.space.create_text(pt[1] - r - 40, pt[0] - r - 10,
                               # anchor = tk.W, text =f"{leg_name_abr}:{FSM.state_str(leg.fsm)}:{round(leg.plan.adj[2], 3)}:{round(leg.position[2], 3)}")

    def draw_circle(self, pos, color):
        r=6
        self.space.create_oval(
            pos[1] - r, pos[0] - r, pos[1] + r, pos[0] + r, fill=color)

    def draw_circle2(self, sp: SPoint):
        r = 6
        self.space.create_oval(sp.x - r, sp.y - r, sp.x +
                               r, sp.y + r, fill=sp.color)

    def draw_force(self):
        q = self.q
        lpt = self.cntr
        reduced = self.mul * q.sens_info.tf_pos + lpt
        self.space.create_line(
            lpt[1], lpt[0], reduced[1], reduced[0], width=4, fill="black")
        self.space.pack()
        self.draw_circle(reduced, q.sens_info.tf_col)
        # self.space.create_text(reduced[1] - 16, reduced[0] - 26,
                # anchor=tk.W, text=f"{round(q.sens_info.avg_leg_h, 3)}:{round(q.sens_info.abs_std_leg_h, 3)}")

    def draw_leg_adjust(self, leg):
        cpt = self.cntr
        lpt = self.mul * leg.position + self.cntr
        reduced = self.mul * self.q.sens_info.tf_pos + self.cntr

        lk, lb = line_cof(cpt[1], cpt[0], lpt[1], lpt[0])
        rk = -1 / lk
        rb = reduced[0] - (reduced[1] * rk)

        _, ix, iy = intersect(lk, lb, rk, rb)

        if (cpt[0] < lpt[0] and cpt[0] < iy) or (cpt[0] > lpt[0] and cpt[0] > iy):
            color = "green"
        else:
            color = "red"

        self.space.create_line(
            ix, iy, reduced[1], reduced[0], width=4, fill="red")
        self.space.create_line(
            lpt[1], lpt[0], cpt[1], cpt[0], width=4, fill="black")
        # self.space.create_line(0, rb, self.width, rk *
                               # self.width+rb, width=4, fill="orange")
        self.space.create_line(
            ix, iy, cpt[1], cpt[0], width=4, fill=color)
        self.space.pack()

    def draw_perimeter(self):
        sens = self.q.sens_info
        t_legs = [l for i, l in enumerate(self.q.legs_cw) if sens.touch_force[l.idx] > 0]

        for i in range(len(t_legs)):
            lpt = self.mul * t_legs[i - 1].position + self.cntr
            cpt = self.mul * t_legs[i].position + self.cntr

            self.space.create_line(
                lpt[1], lpt[0], cpt[1], cpt[0], width=4, fill="green")
            self.space.pack()

        g_legs = [l for i, l in enumerate(self.q.legs_cw) if l.do_balance]

        for i in range(len(g_legs)):
            lpt = self.mul * g_legs[i - 1].position + self.cntr
            cpt = self.mul * g_legs[i].position + self.cntr

            self.space.create_line(
                lpt[1], lpt[0], cpt[1], cpt[0], width=1, fill="green")
            self.space.pack()

    def draw_s_sines(self):
        sc = self.mul * self.q.sens_info.s_center + self.cntr
        scl = self.mul * self.q.sens_info.to_s_closest + self.cntr
        f = self.mul * self.q.sens_info.tf_pos + self.cntr
        self.space.create_line(f[1], f[0], scl[1], scl[0], width=4, fill="blue")
        self.space.create_line(f[1], f[0], sc[1], sc[0], width=4, fill="blue")
        self.space.pack()

    def draw_tasks(self):
        r = 15
        for t in self.tasks:
            if t.idx == -1:
                pos_lst = [l.position for l in self.q.legs if l.do_balance]
                if len(pos_lst) > 0:
                    base_pos = np.average(np.array(pos_lst), axis=0)
                else:
                    base_pos = np.array([0.0, 0.0, 0.0])

                l_pos = np.array([0.0, 0.0, 0.0])
                
                # draw direction
                mid = self.cntr
                pt = self.mul * t.direction + mid[:2]
                self.space.create_line(
                    mid[1], mid[0], pt[1], pt[0], width=3, fill="purple")
                self.draw_circle(pt, "purple")

            else:
                l_pos = self.q.legs[t.idx].position 
                base_pos = np.array([0.0, 0.0, 0.0])

            color = "red" if t.do_lift else "blue"

            last_pt = self.mul * (l_pos + base_pos ) * \
                np.array([1, 1, 1]) + \
                self.cntr

            if t.active:
                self.space.create_oval(last_pt[1] - r, last_pt[0] - r,
                                       last_pt[1] + r, last_pt[0] + r, width=3, outline=color)

            for p in t.points:
                pt = self.mul * (p.pos + base_pos) * \
                    np.array([1, 1, 1]) + \
                    self.cntr

                self.space.create_line(
                    last_pt[1], last_pt[0], pt[1], pt[0], width=3, fill=color)
                self.draw_circle(pt, color)

                last_pt = pt

        self.space.pack()

    def send_go_cmd(self):
        # while self.q_to.full():
            # print("GV queue is full, sleeping...")
            # time.sleep(0.05)

        print("Go")
        self.__clear_dummies()
        # saved_adj = [(np.array([s.y, s.x, -0.0 * self.mul]) - np.array(
        # [self.height / 2, self.width / 2, 0])) / self.mul for s in self.saved]
        # self.q_to.put(self.task_cmd)
        for t in self.tasks:
            t.active = False
            t.dummy = True

    def send_clear_cmd(self):
        # while self.q_cmd.full():
        # print("GV cmd queue is full, sleeping...")
        # time.sleep(0.05)

        print("clear")
        self.__clear_dummies()

        for t in self.tasks:
            t.clear()
            # if t.idx == -1:
                # t.add_pt(GoPoint(np.zeros(3, dtype=float)))
            # else:
                # t.add_pt(GoPoint(self.q.legs[t.idx].def_pos))
            if t.idx != -1:
                t.add_pt(GoPoint(self.q.legs[t.idx].def_pos))

        # self.q_cmd.put(ActCmd.CLEAR)

        self.send_go_cmd()

    def on_plot_btn_clk(self):
        if self.running:
            print("stop")
            os.write(self.vc_fd, bytes(json.dumps({"act": "bye", "data": "empty"}), "utf-8"))
        else:
            print("start")
            os.write(self.vc_fd, bytes(json.dumps({"act": "hello", "data": "empty"}), "utf-8"))

        self.running = not self.running

    def __clear_dummies(self):
        for t in self.tasks:
            if t.dummy:
                t.clear()


    def on_btn_clk(self, ev):
        self.__clear_dummies()
        err = 10

        if self.sel_mode.get() == 1:
            pos = (np.array([ev.y, ev.x]) - np.array(
                [self.height / 2, self.width / 2])) / self.mul

            ct = [t for t in self.tasks if t.idx == -1][0]
            ct.direction= pos
            return

        #  Returning on existing point or leg hit
        for t in self.tasks:
            if t.idx == -1:
                l_pos = np.array([0.0, 0.0, 0.0])
            else:
                l_pos = self.q.legs[t.idx].position 

            s = self.mul * l_pos + self.cntr

            if abs(s[1] - ev.x) <= err and abs(s[0] - ev.y) <= err:
                return

            for p in t.points:
                s = self.mul * p.pos + self.cntr

                if abs(s[1] - ev.x) <= err and abs(s[0] - ev.y) <= err:
                    return

        for t in self.tasks:
            if t.active:
                pos = (np.array([ev.y, ev.x, 0.0]) - np.array(
                    [self.height / 2, self.width / 2, 0.0])) / self.mul

                if t.idx != -1:
                    pos[2] = self.q.legs[t.idx].def_pos[2]

                t.add_pt(GoPoint(pos))
                t.exp_set = True

    def on_btn_dbl_clk(self, ev):
        self.__clear_dummies()
        # print("dbl clicked")
        err = 10

        for t in self.tasks:
            if t.idx == -1:
                l_pos = np.array([0.0, 0.0, 0.0])
            else:
                l_pos = self.q.legs[t.idx].position 

            s = self.mul * l_pos + self.cntr

            if abs(s[1] - ev.x) <= err and abs(s[0] - ev.y) <= err:
                if not t.active:
                    t.active = True

                    # disabling rest of tasks
                    for ts in self.tasks:
                        if ts.idx != t.idx:
                            ts.active = False
                else:
                    t.toggle_lift()

            for j, p in enumerate(t.points):
                s = self.mul * p.pos + self.cntr

                if abs(s[1] - ev.x) <= err and abs(s[0] - ev.y) <= err:
                    t.points.pop(j)
                    if len(t.points) == 0:
                        t.exp_set = False

    def on_got_msg(self, arg1, arg2):
        print(f"GOT MSG {arg1} : {arg2}")
        msg = os.read(self.cv_fd, 2048)
        print(f"msg: {msg}")
        try:
            json_msg = json.loads(msg)
        except:
            print("Failed to parse")
            return 

        act = json_msg.get("act")
        data = json_msg.get("rsp")

        if act is None or data is None:
            print("Failed to parse")
            return 

        # try:
        if act == "vstep":
            sens = VSens(data["touch_force"], data["damp"], data["tf_pos"], data["tf_type"])
            raw_legs = data["legs"]
            legs = [VLeg(l["bal"], l["idx"], l["name"], l["pos"], l["def_pos"]) for l in raw_legs]
            self.q = VQuad(legs, sens)
            self.update()
            os.write(self.vc_fd, bytes(json.dumps({"act":"vstep", "data":{}}), "utf-8"))
        elif act == "hello":
            os.write(self.vc_fd, bytes(json.dumps({"act":"vstep", "data":{}}), "utf-8"))


        # except Exception as e:
            # print(f"Failed to parse: {e}")


    def update(self):
        self.root.update()

        self.space.delete("all")

        self.draw_circle(np.array([self.height / 2, self.width / 2]), "black")
        self.draw_tasks()
        self.draw_perimeter()

        self.draw_legs_middle()
        for l in self.q.legs:
            self.draw_leg_circle(l)
            self.draw_leg_adjust(l)

        # self.draw_s_sines()
        self.draw_force()

    def setup(self):
        pass


if __name__ == "__main__":
    gv = GrView()
    gv.setup()
    gv.root.mainloop()