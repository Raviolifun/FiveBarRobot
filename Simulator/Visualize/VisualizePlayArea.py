import numpy as np
import matplotlib.pylab as plt
import Simulator.FiveBarDynamics as FiveBarDynamics
import math

mass_matrix = [.048, .048, .074 + 0.050, .074 + 0.050]
length_matrix = [.163, .163, .450, .450]
a_length_matrix = [.335, .335, .6025, .6025]
bottom_length = 0.4925

# mass_matrix, length_matrix
dynamics = FiveBarDynamics.Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, 0.35, 0.38, 0)
dynamics.g = 0

# Initialize the x and y matrices
x_res = 1000
y_res = 1000

x, y = np.meshgrid(np.linspace(-1, 1 + bottom_length, x_res), np.linspace(-1, 1, y_res))
z = np.zeros((x_res, y_res))

for i in range(len(x)):
    for j in range(len(x[i])):
        val = dynamics.get_closest_solution(x[i, j], y[i, j], offset=0.01)
        distance = math.sqrt((val[0] - x[i, j])**2 + (val[1] - y[i, j])**2)

        # Set it up such that it shows where the edges of the graph are.
        if distance != 0:
            # z[i, j] = 1/(distance+0.1) + distance**2
            z[i, j] = distance
        else:
            z[i, j] = 0


# x and y are bounds, so z should be the value *inside* those bounds.
# Therefore, remove the last value from the z array.
z = z[:-1, :-1]
z_min, z_max = np.abs(z).min(), np.abs(z).max()

fig, ax = plt.subplots()

c = ax.pcolormesh(x, y, z, cmap=plt.cm.get_cmap('bone').reversed(), vmin=z_min, vmax=z_max)
ax.set_title('Play Area Visualization')
# set the limits of the plot to the limits of the data
ax.axis([x.min(), x.max(), y.min(), y.max()])
plt.ylabel("$y$ $(m)$")
plt.xlabel("$x$ $(m)$")
fig.colorbar(c, ax=ax)

plt.show()
# plt.savefig('Save/VisualizePlayAreaDistance')
