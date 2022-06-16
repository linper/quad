from interp import line_cof, sect_intersect, dist
from plan import DestPoint


class Obstacle:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.perp = x1 == x2

    def hits(self, pt: DestPoint):
        k, b = line_cof(self.x1, self.y1, self.x2, self.y2)
        return pt.x * k + b


class Box:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def get_side(self, id):
        if id == 0:
            return self.x1, self.y1, self.x1, self.y2
        elif id == 1:
            return self.x1, self.y2, self.x2, self.y2
        elif id == 2:
            return self.x2, self.y2, self.x2, self.y1
        elif id == 3:
            return self.x2, self.y1, self.x1, self.y1
        else:
            return None

    def hits(self, pt: DestPoint, r=0.1):
        for i in range(4):
            inter, x, y = sect_intersect(pt.x, pt.y, pt.x, pt.y - r, *self.get_side(i))
            if inter:
                return dist(pt.x, pt.y, x, y)

        return -1
        # return self.x1 < pt.x - r and self.x2 > pt.x + r and self.y1 < pt.y - r and self.y2 > pt.y + r
