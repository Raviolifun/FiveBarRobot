import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
import Simulator.FiveBarDynamics as FiveBarDynamics
import Controller.PathGenerator as PathGenerator
import math


# initial parameters
mass_matrix = [.048, .048, .074+0.050, .074+0.050]
length_matrix = [.163, .163, .450, .450]
a_length_matrix = [.3265, .3265, .580, .580]
bottom_length = 0.6

# mass_matrix, length_matrix
dynamics = FiveBarDynamics.Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, 0.35, 0.38, 0)
dynamics.g = 0

path = PathGenerator.Path(PathGenerator.draw_line(dynamics, 0.35, 0.38, 0, 1, 1, 10, 0, 10))


