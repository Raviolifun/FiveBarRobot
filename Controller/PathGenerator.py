import numpy as np
from scipy.interpolate import interp1d


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

    for i in range(resolution):
        [x[i], y[i]] = dynamics.get_closest_solution(x[i], y[i])

    return PathSegment(x, y, t)


class PathSegment:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t


class Path:
    def __init__(self, list_of_path_segments):
        if len(list_of_path_segments) == 1:
            self.x = list_of_path_segments[0].x
            self.y = list_of_path_segments[0].y
            self.t = list_of_path_segments[0].t
        else:
            self.x = np.array([])
            self.y = np.array([])
            self.t = np.array([])

            for i in range(len(list_of_path_segments)):
                self.x = np.concatenate((self.x, list_of_path_segments[i].x), axis=None)
                self.y = np.concatenate((self.y, list_of_path_segments[i].y), axis=None)
                self.t = np.concatenate((self.t, list_of_path_segments[i].t), axis=None)

        self.interp_function_x = interp1d(self.t, self.x)
        self.interp_function_y = interp1d(self.t, self.y)

    def get_xy_at_time(self, time):
        if time < self.t[0]:
            return [self.x[0], self.y[0]]
        if time > self.t[-1]:
            return [self.x[-1], self.y[-1]]
        else:
            x = self.interp_function_x(time)
            y = self.interp_function_y(time)
            return [x, y]

    def check_smoothness(self, error):
        pass
