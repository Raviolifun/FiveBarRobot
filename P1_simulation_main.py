import P1_dynamics
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import math

matplotlib.use('TkAgg')

if __name__ == '__main__':
    save = False  # Whether it saves the graphs

    tspan = np.linspace(0, 40, 1000)
    number = 1

    def make_plot(name, state):
        plt.figure(name)
        plt.clf()
        plt.title(state)
        plt.ylabel(state)
        plt.xlabel("$t$ $(sec)$")
        plt.grid()

    name_state_dict = {
        "q1": "q1(t)",
        "q2": "q2(t)",
        "q3": "q3(t)",
        "q4": "q4(t)"
    }

    for key in name_state_dict:
        make_plot(key, name_state_dict[key])

    save_total_input       = [None] * number
    save_feedback_errors   = [None] * number
    save_feedforward_input = [None] * number

    for i in range(number):
        print(i)

        mass_matrix = [1, 1, 1, 1]
        length_matrix = [1, 1, 1, 1, 1]

        # mass_matrix, length_matrix
        dynamics = P1_dynamics.Dynamics(mass_matrix, length_matrix)

        q10 = math.radians(108)
        q40 = math.radians(72)
        q20 = math.radians(180 - 72 - 72)
        q30 = math.radians(360 - 108 - 108)

        # q1, q2, q3, q4, qd1, qd2, qd3, qd4, T1, T4
        yinit = [q10, q20, q30, q40, 0, 0, 0, 0, 1, 1]

        # Solve differential equation
        # sol = solve_ivp(lambda t, y: dynamics.f(t, y),
        #                 [tspan[0], tspan[-1]], yinit, t_eval=tspan, rtol=1e-5)
        sol = solve_ivp(lambda t, y: dynamics.f(t, y),
                        [tspan[0], tspan[-1]], yinit, t_eval=tspan)

        if sol.status < 0:
            print(sol.message)

        time = sol.t
        [q1, q2, q3, q4, qd1, qd2, qd3, qd4, T1, T4] = sol.y

        plt.figure("q1")
        plt.plot(time, q1)

        plt.figure("q2")
        plt.plot(time, q2)

        plt.figure("q3")
        plt.plot(time, q3)

        plt.figure("q4")
        plt.plot(time, q4)

    for key in name_state_dict:
        plt.figure(key)
        if save:
            plt.savefig('Saves/' + key)
        else:
            plt.show()

