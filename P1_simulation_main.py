import P1_dynamics
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
    save = False  # Whether it saves the graphs

    tspan = np.linspace(0, 40, 10000)
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
        print(i)

        # initial parameters
        mass_matrix = [.048, .048, .074+0.050, .074+0.050]
        length_matrix = [.163, .163, .450, .450]
        a_length_matrix = [.3265, .3265, .580, .580]
        bottom_length = 0.365

        # initial conditions
        q10 = -math.radians(90)
        q20 = -math.radians(90)

        # mass_matrix, length_matrix
        dynamics = P1_dynamics.Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, False)
        dynamics.g = 0

        # q1, q2, q3, q4, qd1, qd2, qd3, qd4, T1, T4
        yinit = [q10, q20, 0, 0, 0, 1]

        # Solve differential equation
        # sol = solve_ivp(lambda t, y: dynamics.f(t, y),
        #                 [tspan[0], tspan[-1]], yinit, t_eval=tspan, rtol=1e-5)
        sol = solve_ivp(lambda t, y: dynamics.f(t, y), [tspan[0], tspan[-1]], yinit, t_eval=tspan, rtol=1e-5)

        if sol.status < 0:
            print(sol.message)

        time = sol.t
        [q1, q2, qd1, qd2, t1, t4] = sol.y

        def compute_end_effectors(q1, q2):
            q3, q4 = dynamics.get_q3_q4(q1, q2)

            ex1 = math.cos(q1) * dynamics.a1
            ey1 = math.sin(q1) * dynamics.a1

            ex2 = math.cos(q2) * dynamics.a2 + dynamics.LB
            ey2 = math.sin(q2) * dynamics.a2

            ex3 = ex1 + math.cos(q1 + q3) * dynamics.a3
            ey3 = ey1 + math.sin(q1 + q3) * dynamics.a3

            return ex1, ey1, ex2, ey2, ex3, ey3, q3, q4


        func_dyn_vector = np.vectorize(compute_end_effectors)

        # TODO note what these values are
        ex1, ey1, ex2, ey2, ex3, ey3, q3, q4 = func_dyn_vector(q1, q2)

        plt.figure("e3")
        points = np.array([ex3, ey3]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        norm = plt.Normalize(time.min(), time.max())
        lc = LineCollection(segments, cmap='viridis', norm=norm)
        # Set the values used for colormapping
        lc.set_array(time)
        lc.set_linewidth(2)
        line = plt.gca().add_collection(lc)
        plt.colorbar(line, ax=plt.gca())
        plt.gca().set_xlim(ex3.min(), ex3.max())
        plt.gca().set_ylim(ey3.min(), ey3.max())

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

        interp_function_x1 = interp1d(time, ex1)
        interp_function_y1 = interp1d(time, ey1)
        interp_function_x2 = interp1d(time, ex2)
        interp_function_y2 = interp1d(time, ey2)
        interp_function_x3 = interp1d(time, ex3)
        interp_function_y3 = interp1d(time, ey3)

        interp_function_q1 = interp1d(time, q1 * 180 / math.pi)
        interp_function_q2 = interp1d(time, q2 * 180 / math.pi)
        interp_function_q3 = interp1d(time, q3 * 180 / math.pi)
        interp_function_q4 = interp1d(time, q4 * 180 / math.pi)

        ex_min, ex_max = ex3.min(), ex3.max()
        ey_min, ey_max = ey3.min(), ey3.max()

        box_style = BoxStyle("Round", pad=0)

        def animate(i):
            time = i * interval

            x.append(interp_function_x3(time))
            y.append(interp_function_y3(time))

            ax.clear()
            ax.plot(x, y)
            ax.set_aspect('equal')
            ax.set_xlim(ex_min - 1, ex_max + 1)
            ax.set_ylim(ey_min - 1, ey_max + 1)

            # Joint 1
            ts = ax.transData
            coords = ts.transform([0, 0])
            tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], interp_function_q1(time))
            t = ts + tr
            rect1 = Rectangle((0, 0), dynamics.a1, dynamics.a1 / 50, transform=t)
            ax.add_patch(rect1)

            # Joint 2
            coords = ts.transform([dynamics.LB, 0])
            tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], interp_function_q2(time))
            t = ts + tr
            rect1 = Rectangle((dynamics.LB, 0), dynamics.a2, dynamics.a2 / 50, transform=t)
            ax.add_patch(rect1)

            # Joint 3
            coords = ts.transform([interp_function_x1(time), interp_function_y1(time)])
            tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], interp_function_q1(time) + interp_function_q3(time))
            t = ts + tr
            rect1 = Rectangle((interp_function_x1(time), interp_function_y1(time)), dynamics.a3, dynamics.a3 / 50, transform=t)
            ax.add_patch(rect1)

            # Joint 4
            coords = ts.transform([interp_function_x2(time), interp_function_y2(time)])
            tr = matplotlib.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], interp_function_q2(time) + interp_function_q4(time))
            t = ts + tr
            rect1 = Rectangle((interp_function_x2(time), interp_function_y2(time)), dynamics.a4, dynamics.a4 / 50, transform=t)
            ax.add_patch(rect1)


        # run the animation
        ani = FuncAnimation(fig, animate, frames=int(time.max()/interval) - 1, interval=interval*1000, repeat=False)

        if save:
            f = r"C:\Users\ravio\Documents\2_School\School 2022-2023\1_Spring\Honors Thesis\FiveBarRobot\Saves\animation.gif"
            writergif = PillowWriter(fps=30)
            ani.save(f, writer=writergif)
        else:
            plt.show()

