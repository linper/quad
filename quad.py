from leg import Leg


class Quad:
    def __init__(self, model, fll, frl, bll, brl, sensor):
        self.model = model
        self.sensor = sensor
        self.front_left_leg: Leg = fll
        self.front_right_leg: Leg = frl
        self.back_left_leg: Leg = bll
        self.back_right_leg: Leg = brl

        self.legs = [fll, frl, bll, brl]
        self.sensors = [fll.sensor, frl.sensor, bll.sensor, brl.sensor, sensor]

        for l in self.legs:
            l.body = self
