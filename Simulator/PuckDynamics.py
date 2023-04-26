import numpy as np


class Dynamics:
    def __init__(self, mass, radius, friction, restitution_coefficient, px, py,
                 width, length, x_corner, y_corner, stick_thresh=0.02):
        self.mass = mass
        self.radius = radius
        self.friction = friction
        self.restitution_coefficient = restitution_coefficient
        self.px = px
        self.py = py
        self.width = width
        self.length = length
        self.x_corner = x_corner  # Bottom left corner
        self.y_corner = y_corner  # Bottom left corner
        self.stick_thresh = stick_thresh

    def f(self, t, y, dt=0.1):
        # Initialize useful variables
        x, y, xd, yd = y

        print("x, y, xd, yd")
        print([x, y, xd, yd])

        x_bounce = False
        y_bounce = False

        # Detect if we hit a wall, this detection needs to be latching
        if x + self.radius > self.width + self.x_corner:
            # Detect if we smack an x wall
            if xd > 0:
                xd = -self.restitution_coefficient * xd
                xdd = -(-self.restitution_coefficient * xd - xd) / dt
                x_bounce = True

        elif x - self.radius < self.x_corner:
            # Detect if we smack an -x wall
            if xd < 0:
                xd = -self.restitution_coefficient * xd
                xdd = -(-self.restitution_coefficient * xd - xd) / dt
                x_bounce = True

        elif y + self.radius > self.length + self.y_corner:
            # Detect if we smack a y wall
            if yd > 0:
                yd = -self.restitution_coefficient * yd
                ydd = -(-self.restitution_coefficient * yd - yd) / dt
                y_bounce = True

        elif y - self.radius < self.y_corner:
            # Detect if we smack a -y wall
            if yd < 0:
                yd = -self.restitution_coefficient * yd
                ydd = -(-self.restitution_coefficient * yd - yd) / dt
                y_bounce = True

        # If no wall hit, leave answer as is.

        # Then calculate where we should be heading in terms of acceleration and velocity now
        v = np.array([[xd], [yd]])
        v_mag = np.linalg.norm(v)

        v_hat = v/v_mag

        # Calculate the friction now
        friction_vec = - self.friction / self.mass * v_hat

        # Calculate the dynamics, if in sticking threshold, stop moving.
        if v_mag > self.stick_thresh:
            if not x_bounce:
                xdd = self.px / self.mass + friction_vec[0][0]
            if not y_bounce:
                ydd = self.py / self.mass + friction_vec[1][0]
        else:
            print("test_no")
            xdd = 0
            ydd = 0
            xd = 0
            yd = 0

        print("xd, yd, xdd, ydd")
        print([xd, yd, xdd, ydd])

        return [xd, yd, xdd, ydd]
