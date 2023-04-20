import FiveBarDynamics
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyBboxPatch, BoxStyle, Rectangle
import Controller.PathGenerator as PathGenerator
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import math

matplotlib.use('TkAgg')

if __name__ == '__main__':
    save = False  # Whether it saves the graphs or not

    tspan = np.linspace(0, 10, 10000)
    number = 1

    def make_plot(name, state):
        plt.figure(name)
        plt.clf()
        # plt.title(state)
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

        initial_x = 0.9065
        initial_y = 0
        initial_state = 0

        # mass_matrix, length_matrix
        dynamics = FiveBarDynamics.Dynamics(bottom_length, mass_matrix, length_matrix, a_length_matrix, initial_x, initial_y, initial_state)
        dynamics.g = 0

        # Set the path to follow
        path = PathGenerator.Path([PathGenerator.draw_line(dynamics, initial_x, initial_y, 0, 0.5, 0.5, 1, 0, 10), PathGenerator.draw_line(dynamics, 0.5, 0.5, 1, -0.25, 0.25, 2, 0, 10)])
        dynamics.set_path(path)

        # initial conditions
        in_angles = dynamics.get_hint_angle()[0]
        q10 = in_angles[0]
        q20 = in_angles[1]

        # q1, q2, qd1, qd2, T1, T4
        yinit = [q10, q20, 0, 0, 0, 0]

        # Solve differential equation
        sol = solve_ivp(lambda t, y: dynamics.f(t, y), [tspan[0], tspan[-1]], yinit, t_eval=tspan, rtol=5e-3)

        # If it is not able to solve the equation, print the error
        if sol.status < 0:
            print(sol.message)

        # Extract the data of the sim
        time = sol.t
        [q1, q2, qd1, qd2, t1, t4] = sol.y

        # Recompute values. These are not saved in real time because they are occasionally garbage.
        def compute_end_effectors(q1, q2, iteration):
            # Only on the first iteration do we provide the hint:
            if iteration != 0:
                q3, q4 = dynamics.get_q3_q4(q1, q2)
            else:
                q3, q4 = dynamics.get_q3_q4(q1, q2, initial_x, initial_y, initial_state)

            ex1 = math.cos(q1) * dynamics.a1
            ey1 = math.sin(q1) * dynamics.a1

            ex2 = math.cos(q2) * dynamics.a2 + dynamics.LB
            ey2 = math.sin(q2) * dynamics.a2

            [x1, x2, y1, y2] = dynamics.forward_kinematics(q1, q2, q3, q4)

            ex3 = x1
            ey3 = y1

            return ex1, ey1, ex2, ey2, ex3, ey3, q3, q4

        # Recalculate each of the above values:
        func_dyn_vector = np.vectorize(compute_end_effectors)

        # (ex1, ey1) is the end position of link 1
        # (ex2, ey2) is the end position of link 2
        # (ex3, ey3) is the end position of link 3
        # q3, q4 are the angles of link 3 and 4
        ex1, ey1, ex2, ey2, ex3, ey3, q3, q4 = func_dyn_vector(q1, q2, range(len(q1)))

        x_desired = path.x
        y_desired = path.y

        plt.figure("e3")
        points = np.array([ex3, ey3]).T.reshape(-1, 1, 2)
        plt.plot(x_desired, y_desired,  linestyle='dashed', color="red")
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # TODO I do no t think the color mapping over time is working correctly
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

        ###############################################################
        #                          Animation                          #
        ###############################################################

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

        # My current guess is that my animation is screwing up the rotation matrices at some point...
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

