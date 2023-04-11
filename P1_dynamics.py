import math
import numpy as np
# Based all the calculations off a paper titled
# "A Validation Study of PD Control of a Closed-Chain Mechanical System" by Fathi Ghorbel and Ruvinda Gunawardana

#     E
#    / \
#   q3 q4
#   |   |
#  q1   q2
# Shows the numbering system and where each angle is located. Reference the paper for more information


class Dynamics:
    def __init__(self, bottom_length, mass_matrix, length_matrix, a_length_matrix, x_hint, y_hint, state_hint, i_matrix=None):
        # mass
        self.M1 = mass_matrix[0]
        self.M2 = mass_matrix[1]
        self.M3 = mass_matrix[2]
        self.M4 = mass_matrix[3]

        # Distance to center of mass from base joint
        self.L1 = length_matrix[0]
        self.L2 = length_matrix[1]
        self.L3 = length_matrix[2]
        self.L4 = length_matrix[3]

        # Lengths of each bar
        self.LB = bottom_length
        self.a1 = a_length_matrix[0]
        self.a2 = a_length_matrix[1]
        self.a3 = a_length_matrix[2]
        self.a4 = a_length_matrix[3]

        # Position hints for setting initial q3 and q4 angles to correct position
        self.x_hint = x_hint
        self.y_hint = y_hint
        self.state_hint = state_hint

        # Desired start position:
        hint_angle_options = self.inverse_kinematics(self.x_hint, self.y_hint)

        if hint_angle_options:
            hint_angles = hint_angle_options[self.state_hint]
            self.q1_hint = hint_angles[0] % (2 * math.pi)
            self.q2_hint = hint_angles[1] % (2 * math.pi)
            self.q3_hint = hint_angles[2] % (2 * math.pi)
            self.q4_hint = hint_angles[3] % (2 * math.pi)
        else:
            raise ValueError("x and y hint position are unreachable")

        def i_calc(m, a, length):
            return 1/12 * m * a * a + m * length * length

        if i_matrix is not None:
            self.I1 = i_matrix[0]
            self.I2 = i_matrix[1]
            self.I3 = i_matrix[2]
            self.I4 = i_matrix[3]
        else:
            self.I1 = i_calc(self.M1, self.a1, self.L1)
            self.I2 = i_calc(self.M2, self.a2, self.L2)
            self.I3 = i_calc(self.M3, self.a3, self.L3)
            self.I4 = i_calc(self.M4, self.a4, self.L4)

        self.g = 0

    def get_q3_q4(self, q1, q2, x_hint=None, y_hint=None, state_hint=None):
        """
        Given known information about the five bar mechanism, find q1 and q2 to meet those criteria
        Must provide hints or a solution will be chosen based off the last inputs the object has been given.
        :param q1: Angle q1 of the robot mechanism
        :param q2: Angle q2 of the robot mechanism
        :param x_hint: If filled, this will be the hint position that is used when determining a solution
        :param y_hint: If filled, this will be the hint position that is used when determining a solution
        :param state_hint: Determines which of the 4 states will be used for the hint angles
        :return: Returns a list of q3 and q4 given the above information
        """

        if x_hint is not None and y_hint is not None and state_hint is not None:
            hint_angle_options = self.inverse_kinematics(self.x_hint, self.y_hint)

            if hint_angle_options:
                hint_angles = hint_angle_options[state_hint]
                self.q1_hint = hint_angles[0] % (2 * math.pi)
                self.q2_hint = hint_angles[1] % (2 * math.pi)
                self.q3_hint = hint_angles[2] % (2 * math.pi)
                self.q4_hint = hint_angles[3] % (2 * math.pi)
                self.state_hint = state_hint
            else:
                raise ValueError("x and y hint position are unreachable")

        elif x_hint is not None or y_hint is not None or state_hint is not None:
            raise ValueError("Not enough inputs were provided, must provide at x, y and state or none")

        # Useful pre-calculations
        s_1 = math.sin(q1)
        c_1 = math.cos(q1)
        s_2 = math.sin(q2)
        c_2 = math.cos(q2)

        # ==================================================================================
        # ============ First we must calculate the q3 and q4 joint angles ==================
        u_q1q2 = self.a2 * s_2 - self.a1 * s_1
        y_q1q2 = self.a2 * c_2 - self.a1 * c_1 + self.LB
        c_q1q2 = self.a3**2 - self.a4**2 - y_q1q2**2 - u_q1q2**2
        b_q1q2 = 2 * self.a4 * u_q1q2
        a_q1q2 = 2 * self.a4 * y_q1q2

        # Depending on the operating mode, must choose one of these two equations for q4
        feasibility = a_q1q2**2 + b_q1q2**2 - c_q1q2**2

        # I'm not sure if this switching mechanism is all that good...
        if feasibility < 0:
            # Consider adding flipping logic hear, would involve looking at the angular velocities and carry through
            print("Entered singularity, recoverable?")
            feasibility = 0

        # Find it for first state:
        param = math.sqrt(feasibility)
        q4_1 = math.atan2(param, c_q1q2) + math.atan2(b_q1q2, a_q1q2) - q2

        # using this value of q4, find q3
        q3_1 = math.atan2(u_q1q2 + self.a4 * math.sin(q2 + q4_1), y_q1q2 + self.a4 * math.cos(q2 + q4_1)) - q1

        # Find it for second state:
        param = -math.sqrt(feasibility)
        q4_2 = math.atan2(param, c_q1q2) + math.atan2(b_q1q2, a_q1q2) - q2

        # using this value of q4, find q3
        q3_2 = math.atan2(u_q1q2 + self.a4 * math.sin(q2 + q4_2), y_q1q2 + self.a4 * math.cos(q2 + q4_2)) - q1

        # Normalize them so we are comparing apples to apples
        q4_1 %= (2 * math.pi)
        q3_1 %= (2 * math.pi)
        q4_2 %= (2 * math.pi)
        q3_2 %= (2 * math.pi)

        # Now see which is closer to the previous hint, and then choose that one (for now at least)
        first_con = (q4_1 - self.q4_hint) * (q4_1 - self.q4_hint) + (q3_1 - self.q3_hint) * (q3_1 - self.q3_hint)
        second_con = (q4_2 - self.q4_hint) * (q4_2 - self.q4_hint) + (q3_2 - self.q3_hint) * (q3_2 - self.q3_hint)
        self.q1_hint = q1
        self.q2_hint = q2
        # print("q4h: " + str(self.q4_hint) + ", q4_1: " + str(q4_1) + "q4_2: " + str(q4_2))
        # print("q3h: " + str(self.q3_hint) + ", q3_1: " + str(q3_1) + "q3_2: " + str(q3_2))
        if first_con < second_con:
            # Choose the first condition
            self.q3_hint = q3_1
            self.q4_hint = q4_1
            # print("fc")
            return q3_1, q4_1

        else:
            # choose the second condition
            self.q3_hint = q3_2
            self.q4_hint = q4_2
            # print("sc")
            return q3_2, q4_2

    def f(self, t, y):
        q1, q2, qd1, qd2, t1, t2 = y

        qd = np.array([[qd1], [qd2]])

        # ==================================================================================
        # ==============================      Dynamics      ================================
        # ==================================================================================

        # Useful pre-calculations
        s_1 = math.sin(q1)
        c_1 = math.cos(q1)
        s_2 = math.sin(q2)
        c_2 = math.cos(q2)

        # ==================================================================================
        # ============ First we must calculate the q3 and q4 joint angles ==================

        q3, q4 = self.get_q3_q4(q1, q2)

        # ==================================================================================
        # ================ Time to compute qd_prime from q_prime and qd ====================

        s_13 = math.sin(q1 + q3)
        s_24 = math.sin(q2 + q4)
        c_13 = math.cos(q1 + q3)
        c_24 = math.cos(q2 + q4)

        phi11 = -self.a1 * s_1 - self.a3 * s_13
        phi12 = self.a2 * s_2 + self.a4 * s_24
        phi13 = -self.a3 * s_13
        phi14 = self.a4 * s_24
        phi21 = self.a1 * c_1 + self.a3 * c_13
        phi22 = -self.a2 * c_2 - self.a4 * c_24
        phi23 = self.a3 * c_13
        phi24 = -self.a4 * c_24

        phi1 = np.array([phi11, phi12, phi13, phi14])
        phi2 = np.array([phi21, phi22, phi23, phi24])
        phi3 = np.array([1, 0, 0, 0])
        phi4 = np.array([0, 1, 0, 0])

        phi = np.array([phi1, phi2, phi3, phi4])
        try:
            phi_inv = np.linalg.inv(phi)
        except np.linalg.LinAlgError as err:
            phi_inv = np.linalg.pinv(phi)

        selection_matrix = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])
        rho = np.matmul(phi_inv, selection_matrix)

        qd_prime = np.matmul(rho, qd)

        qd3 = qd_prime[2][0]
        qd4 = qd_prime[3][0]

        # ==================================================================================
        # =================== Time to compute the gravity matrix g_comp ====================

        g_prime = self.g * np.array([[(self.M1 * self.L1 + self.M3 * self.a1) * c_1 + self.M3 * self.L3 * c_13],
                                     [(self.M2 * self.L2 + self.M4 * self.a2) * c_2 + self.M4 * self.L4 * c_24],
                                     [self.M3 * self.L3 * c_13], [self.M4 * self.L4 * c_24]])
        rho_t = np.transpose(rho)
        g_comp = np.matmul(rho_t, g_prime)

        # ==================================================================================
        # =================== Time to compute the inertia matrix d_comp ====================

        c_3 = math.cos(q3)
        c_4 = math.cos(q4)

        d11 = self.M1 * self.L1**2 + self.M3 * (self.a1**2 + self.L3**2 + 2 * self.a1 * self.L3 * c_3) + self.I1 + self.I3
        d13 = self.M3 * (self.L3**2 + self.a1 * self.L3 * c_3) + self.I3
        d22 = self.M2 * self.L2**2 + self.M4 * (self.a2**2 + self.L4**2 + 2 * self.a2 * self.L4 * c_4) + self.I2 + self.I4
        d24 = self.M4 * (self.L4**2 + self.a2 * self.L4 * c_4) + self.I4
        d31 = d13
        d33 = self.M3 * self.L3**2 + self.I3
        d42 = d24
        d44 = self.M4 * self.L4**2 + self.I4

        d1 = np.array([d11, 0, d13, 0])
        d2 = np.array([0, d22, 0, d24])
        d3 = np.array([d31, 0, d33, 0])
        d4 = np.array([0, d42, 0, d44])

        d_prime = np.array([d1, d2, d3, d4])
        d_comp = np.matmul(rho_t, np.matmul(d_prime, rho))

        # ==================================================================================
        # ================== Time to compute the coriolis matrix c_prime ===================

        s_3 = math.sin(q3)
        s_4 = math.sin(q4)

        h1 = -self.M3 * self.a1 * self.L3 * s_3
        h2 = -self.M4 * self.a2 * self.L4 * s_4

        c1 = np.array([h1 * qd3, 0, h1 * (qd1 + qd3), 0])
        c2 = np.array([0, h2 * qd4, 0, h2 * (qd2 + qd4)])
        c3 = np.array([-h1 * qd1, 0, 0, 0])
        c4 = np.array([0, -h2 * qd2, 0, 0])

        c_prime = np.array([c1, c2, c3, c4])

        # ==================================================================================
        # ======================= Time to compute the rho_d matrix =========================

        phi_d11 = -self.a1 * c_1 * qd1 - self.a3 * c_13 * (qd1 + qd3)
        phi_d12 = self.a2 * c_2 * qd2 + self.a4 * c_24 * (qd2 + qd4)
        phi_d13 = -self.a3 * c_13 * (qd1 + qd3)
        phi_d14 = self.a4 * c_24 * (qd2 + qd4)

        phi_d21 = - self.a1 * s_1 * qd1 - self.a3 * s_13 * (qd1 + qd3)
        phi_d22 = self.a2 * s_2 * qd2 + self.a4 * s_24 * (qd2 + qd4)
        phi_d23 = -self.a3 * s_13 * (qd1 + qd3)
        phi_d24 = self.a4 * s_24 * (qd2 + qd4)

        phi_d1 = np.array([phi_d11, phi_d12, phi_d13, phi_d14])
        phi_d2 = np.array([phi_d21, phi_d22, phi_d23, phi_d24])
        phi_d3 = np.array([0, 0, 0, 0])
        phi_d4 = np.array([0, 0, 0, 0])

        phi_d = np.array([phi_d1, phi_d2, phi_d3, phi_d4])
        rho_d = np.matmul(-phi_inv, np.matmul(phi_d, rho))

        # ==================================================================================
        # ====================== Time to compute the c_comp matrix =========================

        c_comp = np.matmul(rho_t, np.matmul(c_prime, rho)) + np.matmul(rho_t, np.matmul(d_prime, rho_d))

        # ==================================================================================
        # ===========================     Controller time     ==============================
        # ==================================================================================

        angles = self.inverse_kinematics(-0.2, 0.65)[0]
        #print(angles)

        # dirt basic controller, I give it desired positions manually from here
        goal_q1 = angles[0]
        goal_q2 = angles[1]

        error_q1 = goal_q1 - q1
        error_q2 = goal_q2 - q2

        error_q1 = error_q1 % (2 * math.pi)
        if abs(error_q1) > math.pi:
            error_q1 = -error_q1

        error_q2 = error_q2 % (2 * math.pi)
        if abs(error_q2) > math.pi:
            error_q2 = -error_q2

        t1 = error_q1 * 5 - qd1 * 2
        t2 = error_q2 * 5 - qd2 * 2

        max_torque = 2

        if t1 > max_torque:
            t1 = max_torque
        elif t1 < -max_torque:
            t1 = -max_torque
        if t2 > max_torque:
            t2 = max_torque
        elif t2 < -max_torque:
            t2 = -max_torque

        print("t1: " + str(t1) + ", t2:" + str(t2))
        # print((t1, t2))

        # ==================================================================================
        # ======================= Time to compute the qdd matrix ===========================

        # This is where we could add reaction forces from a puck or the ground
        t = np.array([[t1], [t2]])

        right_side = (t - np.matmul(c_comp, qd) - g_comp)
        try:
            left_side = np.linalg.inv(d_comp)
        except np.linalg.LinAlgError as err:
            left_side = np.linalg.pinv(d_comp)

        qdd = np.matmul(left_side, right_side)

        qdd1, qdd2 = qdd

        td1 = 0
        td2 = 0

        return [qd1, qd2, qdd1, qdd2, td1, td2]

    def inverse_kinematics(self, x, y):
        """
        Given an x and a y position, outputs all the possible ways for the device to meet these criteria.
        Refer to Honors thesis paper for how the derivation for the inverse kinematics were derived.
        :param x: The x position of the robot in the robot frame
        :param y: The y position of the robot in the robot frame
        :return: A matrix of all the solutions, 0 to 4 in length, [[q1, q2, q3, q4], [q1, q2, q3, q4], [q1, q2, q3, q4], [q1, q2, q3, q4]]
                 If it reports zero, no solution was found, and if there are less than 4, the device is near or on a singularity
        """

        try:
            q4 = math.acos((-self.a4 * self.a4 - self.a2 * self.a2 + (x - self.LB) * (x - self.LB) + y * y) /
                           (2 * self.a4 * self.a2))
            q3 = math.acos((-self.a1 * self.a1 - self.a3 * self.a3 + x * x + y * y) /
                           (2 * self.a1 * self.a3))
        except ValueError:
            return []

        def calc_inside(x, y, state):
            """
            :param x: The desired x parameter
            :param y: The desired y parameter
            :param state: There are 4 possible states the device can be in, select the desired one from 0 to 3
            :return: The q1 and q2 that reach this state and x and y pos
            """

            square_distance_13 = x * x + y * y
            square_distance_24 = (x - self.LB) * (x - self.LB) + y * y

            # If there is no solution, then return nothing
            try:
                alpha1 = math.acos((square_distance_13 + self.a1 * self.a1 - self.a3 * self.a3) /
                                   (2 * math.sqrt(square_distance_13) * self.a1))
                alpha2 = math.acos((square_distance_24 + self.a2 * self.a2 - self.a4 * self.a4) /
                                   (2 * math.sqrt(square_distance_24) * self.a2))
            except ValueError:
                return []

            if state == 0:
                q1 = math.atan2(y, x) - alpha1
                q2 = math.atan2(y, x - self.LB) - alpha2
            elif state == 1:
                q1 = math.atan2(y, x) - alpha1
                q2 = math.atan2(y, x - self.LB) + alpha2
            elif state == 2:
                q1 = math.atan2(y, x) + alpha1
                q2 = math.atan2(y, x - self.LB) - alpha2
            elif state == 3:
                q1 = math.atan2(y, x) + alpha1
                q2 = math.atan2(y, x - self.LB) + alpha2
            else:
                raise ValueError("No state with this ID")

            return [q1, q2]

        ans = calc_inside(x, y, 0)

        if ans:
            [q1, q2] = ans
            ans1 = [q1, q2, q3, q4]

            [q1, q2] = calc_inside(x, y, 1)
            ans2 = [q1, q2, q3, -q4]

            [q1, q2] = calc_inside(x, y, 2)
            ans3 = [q1, q2, -q3, q4]

            [q1, q2] = calc_inside(x, y, 3)
            ans4 = [q1, q2, -q3, -q4]

            return [ans1, ans2, ans3, ans4]

        else:
            return []

    def get_hint_angle(self):
        """
        Gets the current hint angles
        :return: Returns the current hint angles from 1 to 4 in a list and the hint state
        """
        return [self.q1_hint, self.q2_hint, self.q3_hint, self.q4_hint], self.state_hint

    def draw_line(self, x0, y0, t0, x1, y1, t1, resolution):
        """
        Finds a list of angles from position (x0, y0) to (x1, y1) at a given resolution between time instance t0 and t1
        This is a very basic mapping from the xy frame to the angular frame of the robot
        :param x0: Initial x position
        :param y0: Initial y position
        :param t0: Initial time
        :param x1: Final x position
        :param y1: Final y position
        :param t1: Final time
        :param resolution: The number of points that the line is broken into
        :return:
        """
        pass

    def cartesian_distance_to_singularity(self, x, y):
        """
        Given an x and a y position, outputs the cartesian distances to all possible singularities
        :param x: The x position of the robot in the robot frame
        :param y: The y position of the robot in the robot frame
        :return: A matrix of the lengths to each singularity. Should always be of length 4 like so: [L1, L2, L3, L4]

        """
        pass

    def forward_kinematics(self, q1, q2, q3, q4):
        """
        Given the following parameters finds the position of the end effector using both the right and left side of
        the link. It is left to the user to determine if any errors in the output are problematic.
        :param q1: The first angle of the 5 bar
        :param q2: The second angle of the 5 bar
        :param q3: The third angle of the 5 bar
        :param q4: The fourth angle of the 5 bar
        :return: returns a vector of the x and of the y positions of both sides of the device [x1, x2, y1, y2]
        """
        x1 = self.a1 * math.cos(q1) + self.a3 * math.cos(q1 + q3)
        y1 = self.a1 * math.sin(q1) + self.a3 * math.sin(q1 + q3)
        x2 = self.LB + self.a2 * math.cos(q2) + self.a4 * math.cos(q2 + q4)
        y2 = self.a2 * math.sin(q2) + self.a4 * math.sin(q2 + q4)

        return [x1, x2, y1, y2]


