import P1_dynamics
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import math

matplotlib.use('TkAgg')

if __name__ == '__main__':
    save = False  # Whether it saves the graphs

    tspan = np.linspace(0, 5, 1000)
    number = 1

    def make_plot(name, state):
        plt.figure(name)
        plt.clf()
        plt.title(state)
        plt.ylabel("$x(t)$ $(m)$")
        plt.xlabel("$y(t)$ $(m)$")
        plt.grid()

    name_state_dict = {
        "e1": "e1(t)",
        "e2": "e2(t)"
    }

    for key in name_state_dict:
        make_plot(key, name_state_dict[key])

    for i in range(number):
        print(i)

        # initial parameters
        mass_matrix = [1, 1, 1, 1]
        length_matrix = [.5, .5, .5, .5]
        a_length_matrix = [1, 1, 1, 1]
        bottom_length = 1

        # initial conditions
        q10 = math.radians(90)
        q20 = math.radians(90)
        q40 = math.radians(45)

        # mass_matrix, length_matrix
        dynamics = P1_dynamics.Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, q40)

        # q1, q2, q3, q4, qd1, qd2, qd3, qd4, T1, T4
        yinit = [q10, q20, 0, 0, 0, 1]

        # Solve differential equation
        # sol = solve_ivp(lambda t, y: dynamics.f(t, y),
        #                 [tspan[0], tspan[-1]], yinit, t_eval=tspan, rtol=1e-5)
        sol = solve_ivp(lambda t, y: dynamics.f(t, y),
                        [tspan[0], tspan[-1]], yinit, t_eval=tspan)

        if sol.status < 0:
            print(sol.message)

        time = sol.t
        [q1, q2, qd1, qd2, t1, t4] = sol.y

        dynamics.previous_q4 = q40

        def compute_end_effectors(q1, q2):
            q3, q4 = dynamics.get_q3_q4(q1, q2)

            ex1 = math.cos(q2) * dynamics.L2 + math.cos(q2 + q4) * dynamics.L4
            ey1 = math.sin(q2) * dynamics.L2 + math.sin(q2 + q4) * dynamics.L4

            ex2 = math.cos(q1) * dynamics.L1 + math.cos(q1 + q3) * dynamics.L3 + dynamics.LB
            ey2 = math.sin(q1) * dynamics.L1 + math.sin(q1 + q3) * dynamics.L3

            return ex1, ey1, ex2, ey2


        func_dyn_vector = np.vectorize(compute_end_effectors)

        # ex1: end effector pos_x estimate 1
        # ex2: end effector pos_x estimate 2
        # ey1: end effector pos_y estimate 1
        # ey2: end effector pos_y estimate 2
        ex1, ey1, ex2, ey2 = func_dyn_vector(q1, q2)

        plt.figure("e1")
        points = np.array([ex1, ey1]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        norm = plt.Normalize(time.min(), time.max())
        lc = LineCollection(segments, cmap='viridis', norm=norm)
        # Set the values used for colormapping
        lc.set_array(time)
        lc.set_linewidth(2)
        line = plt.gca().add_collection(lc)
        plt.colorbar(line, ax=plt.gca())
        plt.gca().set_xlim(ex1.min(), ex1.max())
        plt.gca().set_ylim(ey1.min(), ey1.max())

        plt.figure("e2")
        points = np.array([ex2, ey2]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        norm = plt.Normalize(time.min(), time.max())
        lc = LineCollection(segments, cmap='viridis', norm=norm)
        # Set the values used for colormapping
        lc.set_array(time)
        lc.set_linewidth(2)
        line = plt.gca().add_collection(lc)
        plt.colorbar(line, ax=plt.gca())
        plt.gca().set_xlim(ex2.min(), ex2.max())
        plt.gca().set_ylim(ey2.min(), ey2.max())

        if save:
            plt.savefig('Saves/' + key)
        else:
            plt.show()

        # create empty lists for the x and y data
        x = []
        y = []

        # create the figure and axes objects
        fig, ax = plt.subplots()
        interval = 1/60

        interp_function_x = interp1d(time, ex1)
        interp_function_y = interp1d(time, ey1)

        ex_min, ex_max = ex1.min(), ex1.max()
        ey_min, ey_max = ey1.min(), ey1.max()

        def animate(i):
            time = i * interval

            x.append(interp_function_x(time))
            y.append(interp_function_y(time))

            ax.clear()
            ax.plot(x, y)
            plt.gca().set_xlim(ex_min, ex_max)
            plt.gca().set_ylim(ey_min, ey_max)

        # run the animation
        ani = FuncAnimation(fig, animate, frames=int(time.max()/interval) - 1, interval=interval*1000, repeat=False)

        plt.show()

