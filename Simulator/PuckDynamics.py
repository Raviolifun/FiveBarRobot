import numpy as np

class Dynamics:
    def __init__(self, mass, radius, friction, restitution_coefficient, px, py, stick_thresh=0.02):
        self.mass = mass
        self.radius = radius
        self.friction = friction
        self.restitution_coefficient = restitution_coefficient
        self.px = px
        self.py = py
        self.stick_thresh = stick_thresh

    def f(self, t, y):
        x, y, xd, yd = y

        v = np.array([[xd], [yd]])
        v_mag = np.linalg.norm(v)

        v_hat = v/v_mag

        friction_vec = - self.friction / self.mass * v_hat

        if v_mag > self.stick_thresh:
            xdd = self.px / self.mass + friction_vec[0][0]
            ydd = self.py / self.mass + friction_vec[1][0]
        else:
            xdd = 0
            ydd = 0
            xd = 0
            yd = 0

        print([xd, yd, xdd, ydd])

        return [xd, yd, xdd, ydd]
