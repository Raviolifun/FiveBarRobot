import numpy as np


def draw_line(dynamics, x0, y0, t0, x1, y1, t1, state, resolution):
    """
    Finds a list of angles from position (x0, y0) to (x1, y1) at a given resolution between time instance t0 and t1
    This is a very basic mapping from the xy frame to the angular frame of the robot
    :param dynamics: the dynamics of choice for the system
    :param x0: Initial x position
    :param y0: Initial y position
    :param t0: Initial time
    :param x1: Final x position
    :param y1: Final y position
    :param t1: Final time
    :param state: If there are multiple options, choose the option specified here. It is suggested to maintain this
                  between path segments, but a smoothing algorithm of some sort could allow chaining of this information
    :param resolution: The number of points that the line is broken into
    :return: Returns a PathSegment object of the x y and t points
    """

    x = np.linspace(x0, x1, resolution)
    y = np.linspace(y0, y1, resolution)
    t = np.linspace(t0, t1, resolution)

    def inverse_kinematics(x, y):
        outputs = dynamics.inverse_kinematics(x, y)

        if outputs:
            return outputs[state]
        else:
            return dynamics.get_closest_solution(x, y)

    func_inv_vector = np.vectorize(dynamics.inverse_kinematics)

    return PathSegment(x, y, t)


class PathSegment:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t


class Path:
    def __init__(self, list_of_segments):
        pass

    def check_smoothness(self, error):
        pass
