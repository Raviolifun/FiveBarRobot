import PuckDynamics
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyBboxPatch, BoxStyle, Rectangle
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import math

matplotlib.use('TkAgg')

if __name__ == '__main__':
    save = False  # Whether it saves the graphs or not

    t_start = 0
    t_end = 10
    resolution = 5000

    tspan = np.linspace(t_start, t_end, resolution)
    number = 1

    def make_plot(name, state):
        plt.figure(name)
        plt.clf()
        plt.title(state)
        plt.ylabel("$y(t)$ $(m)$")
        plt.xlabel("$x(t)$ $(m)$")
        plt.grid()

    name_state_dict = {
        "e3": "e3(t)"
    }

    for key in name_state_dict:
        make_plot(key, name_state_dict[key])

    for i in range(number):
        # mass_matrix, length_matrix
        dynamics = PuckDynamics.Dynamics(0.01, 1.25, 0.01, 0.95, 0, 0, 40, 80, 0, 0)

        # x, y, xd, yd
        yinit = [3, 3, 30, 100]
        #
        # # Solve differential equation
        # sol = solve_ivp(lambda t, y: dynamics.f(t, y), [tspan[0], tspan[-1]], yinit, t_eval=tspan, rtol=5e-3)
        #
        # # If it is not able to solve the equation, print the error
        # if sol.status < 0:
        #     print(sol.message)
        #
        # # Extract the data of the sim
        # time = sol.t
        # [x, y, xd, yd] = sol.y

        # Time to run sim over
        time = tspan

        # Basic solver
        dt = (tspan[-1] - tspan[0]) / len(tspan)

        x = [None] * len(tspan)
        x[0] = yinit[0]
        y = [None] * len(tspan)
        y[0] = yinit[1]
        xd = [None] * len(tspan)
        xd[0] = yinit[2]
        yd = [None] * len(tspan)
        yd[0] = yinit[3]

        for i in range(len(tspan) - 1):
            dh = dynamics.f(tspan[i], [x[i], y[i], xd[i], yd[i]], dt = dt)
            x_increase = dh[0] * dt
            y_increase = dh[1] * dt
            xd_increase = dh[2] * dt
            yd_increase = dh[3] * dt
            x[i + 1] = x_increase + x[i]
            y[i + 1] = y_increase + y[i]
            xd[i + 1] = xd_increase + xd[i]
            yd[i + 1] = yd_increase + yd[i]

        x = np.array(x)
        y = np.array(y)

        plt.figure("Puck Position")
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # TODO I do no t think the color mapping over time is working correctly
        norm = plt.Normalize(time.min(), time.max())
        lc = LineCollection(segments, cmap='viridis', norm=norm)
        # Set the values used for colormapping
        lc.set_array(time)
        lc.set_linewidth(2)
        line = plt.gca().add_collection(lc)
        plt.colorbar(line, ax=plt.gca())
        plt.gca().set_xlim(x.min(), x.max())
        plt.gca().set_ylim(y.min(), y.max())

        if save:
            plt.savefig('../Saves/' + key)
        else:
            plt.show()

        ###############################################################
        #                          Animation                          #
        ###############################################################

        # create empty lists for the x and y data
        x_point = []
        y_point = []

        # create the figure and axes objects
        fig, ax = plt.subplots()
        interval = 1/60

        interp_function_x = interp1d(time, x)
        interp_function_y = interp1d(time, y)

        x_min, x_max = x.min(), x.max()
        y_min, y_max = y.min(), y.max()

        box_style = BoxStyle("Round", pad=0)

        def animate(i):
            time = i

            x_point.append(interp_function_x(time))
            y_point.append(interp_function_y(time))

            ax.clear()
            ax.plot(x_point, y_point)
            ax.set_aspect('equal')
            ax.set_xlim(x_min - 1, x_max + 1)
            ax.set_ylim(y_min - 1, y_max + 1)

        # run the animation
        # TODO, still didn't fix it.
        ani = FuncAnimation(fig, animate, frames=time, interval=(t_end - t_start)/resolution*1000, repeat=False)

        if save:
            f = r"C:\Users\ravio\Documents\2_School\School 2022-2023\1_Spring\Honors Thesis\FiveBarRobot\Saves\animation_test.gif"
            writergif = PillowWriter(fps=30)
            ani.save(f, writer=writergif)
        else:
            plt.show()

