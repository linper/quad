class PIDC:
    def __init__(self, p: float, i: float, d: float, dt: float):
        self.p: float = p
        self.i: float = i
        self.d: float = d
        self.dt: float = dt
        self.it: float = 0.0
        self.perr: float = 0.0

    def eval(self, cur: float, dest: float):
        err = dest - cur
        P = self.p * err
        I = self.it + self.i * err * self.dt
        D = self.d * (err - self.perr) / self.dt

        self.it = I
        self.perr = err

        val = P + I + D

        return val


class PIDC2:
    def __init__(self, px: float, py: float, ix: float, iy: float, dx: float, dy: float, dt: float):
        self.xpid = PIDC(px, ix, dx, dt)
        self.ypid = PIDC(py, iy, dy, dt)

    def eval(self, cur_x: float, cur_y: float, dest_x: float, dest_y: float):
        x_val = self.xpid.eval(cur_x, dest_x)
        y_val = self.ypid.eval(cur_y, dest_y)

        return x_val, y_val
