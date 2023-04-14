import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
import Simulator.P1_dynamics as P1_dynamics
import math

# initial parameters
mass_matrix = [.048, .048, .074+0.050, .074+0.050]
length_matrix = [.163, .163, .450, .450]
a_length_matrix = [.3265, .3265, .580, .580]
bottom_length = 0.6

# mass_matrix, length_matrix
dynamics = P1_dynamics.Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, 0.35, 0.38, 0)
dynamics.g = 0

angle = dynamics.get_hint_angle()[0]
# print(angle)
position = dynamics.forward_kinematics(*angle)
# print(position)

print(dynamics.get_closest_solution(0, 0))
