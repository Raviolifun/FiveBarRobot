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


def get_angle_difference(angle_1, angle_2):
    """
    Finds the minimum angular difference between two angles that are within the domain of 0 to 2*pi
    :param angle_1: First input
    :param angle_2: Second input
    :return: The difference between the two angles. If angle_2 is larger up to pi than angle_1, output is negative
    """

    difference = angle_1 - angle_2

    if difference > math.pi:
        difference = (difference - 2*math.pi)
    elif difference < -math.pi:
        difference = (difference + 2*math.pi)

    return difference


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

        # Path
        self.path = None

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

        # Now see which is closer to the previous hint, and then choose that one
        error4_1 = get_angle_difference(q4_1, self.q4_hint)
        error4_2 = get_angle_difference(q4_2, self.q4_hint)
        error3_1 = get_angle_difference(q3_1, self.q3_hint)
        error3_2 = get_angle_difference(q3_2, self.q3_hint)

        first_con = (error4_1**2) + (error3_1**2)
        second_con = (error4_2**2) + (error3_2**2)
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

        angles = self.inverse_kinematics(*self.path.get_xy_at_time(t))[0]

        # dirt basic controller, I give it desired positions manually from here
        goal_q1 = angles[0]
        goal_q2 = angles[1]

        error_q1 = get_angle_difference(goal_q1, q1)
        error_q2 = get_angle_difference(goal_q2, q2)

        t1 = error_q1 * 150 - qd1 * 5
        t2 = error_q2 * 155 - qd2 * 5

        max_torque = 5

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

        # print(self.forward_kinematics(q1, q2, q3, q4))

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

    def get_closest_solution(self, x, y, offset=0.0):
        """
        Given an x and y coordinate, find a point closest to that point that is a solution to the kinematics
        of the five bar.
        TODO decide what to do if either of the distances are zero
        TODO This does not account for when the base is too far apart, check for big and small circle collision
        :param x: The desired x position
        :param y: The desired y position
        :param offset: an amount to offset a point by, protects against rounding errors
        :return: An array of the x and y position that meet the solution criteria offset by the given amount.
        """
        # find the distance values for the left circle
        left_distance = math.sqrt(x ** 2 + y ** 2)
        left_min_distance = abs(self.a1 - self.a3) + offset
        left_max_distance = abs(self.a1 + self.a3) - offset

        # find the distance values for the right circle
        right_distance = math.sqrt((x - self.LB) ** 2 + y ** 2)
        right_min_distance = abs(self.a2 - self.a4) + offset
        right_max_distance = abs(self.a2 + self.a4) - offset

        if left_distance == 0 or right_distance == 0:
            raise ValueError("Divide by zero error, distance to join cannot be 0. "
                             "This is technically a bug, feel free to fix this by selecting "
                             "a random point from the available points.")

        # Find the intersection of the two circles if it exists
        x1 = (left_min_distance**2 + self.LB**2 - right_min_distance**2) / (2 * self.LB)
        operand = left_min_distance**2 - x1**2

        center_line_small = left_min_distance + (self.LB - left_min_distance - right_min_distance)/2
        center_line_big = (right_max_distance**2 - left_max_distance**2 - self.LB**2) / (-2 * self.LB)

        if operand >= 0:
            # If it exists, do these calcs
            y1 = math.sqrt(left_min_distance**2 - x1**2)

            if left_distance < left_min_distance or right_distance < right_min_distance:
                # If the point is within either of the two centers areas:
                if x < x1:
                    # If on the left side, check if the point is within the intersection triangle
                    cap = y1 / x1 * x
                    if -cap < y < cap:
                        # If true, return the closest intersection point
                        return [x1, math.copysign(y1, y)]
                    else:
                        # If not in the triangle, find the value closest to the circle
                        return [x / left_distance * left_min_distance, y / left_distance * left_min_distance]
                else:
                    # If on the left side, check if the point is within the intersection triangle
                    cap = y1 / (x1 - self.LB) * (x - self.LB)
                    if -cap < y < cap:
                        # If true, return the closest intersection point
                        return [x1, math.copysign(y1, y)]
                    else:
                        return [(x - self.LB) / right_distance * right_min_distance + self.LB, y / right_distance * right_min_distance]

            elif left_distance < left_max_distance and right_distance < right_max_distance:
                # If the point is within both of the two major circles, then define the return value as the same value
                return [x, y]

            else:
                # if the point does not fall within the available area, find the closest outside point
                if x < center_line_big:
                    return [(x - self.LB) / right_distance * right_max_distance + self.LB,
                            y / right_distance * right_max_distance]
                else:
                    return [x / left_distance * left_max_distance, y / left_distance * left_max_distance]

        else:
            if left_distance < left_min_distance or right_distance < right_min_distance:
                # If the point is within either of the two centers areas:
                if x < center_line_small:
                    # If on the left side, check if the point is within the intersection triangle
                    return [x / left_distance * left_min_distance, y / left_distance * left_min_distance]
                else:
                    # If on the left side, check if the point is within the intersection triangle
                    return [(x - self.LB) / right_distance * right_min_distance + self.LB,
                            y / right_distance * right_min_distance]

            elif left_distance < left_max_distance and right_distance < right_max_distance:
                # If the point is within both of the two major circles, then define the return value as the same value
                return [x, y]

            else:
                # if the point does not fall within the available area, find the closest outside point
                if x < center_line_big:
                    return [(x - self.LB) / right_distance * right_max_distance + self.LB,
                            y / right_distance * right_max_distance]
                else:
                    return [x / left_distance * left_max_distance, y / left_distance * left_max_distance]

    def get_hint_angle(self):
        """
        Gets the current hint angles
        :return: Returns the current hint angles from 1 to 4 in a list and the hint state
        """
        return [self.q1_hint, self.q2_hint, self.q3_hint, self.q4_hint], self.state_hint

    def cartesian_distance_to_singularity(self, x, y):
        """
        Given an x and a y position, outputs the cartesian distances to the closest singularity
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

    def set_path(self, path):
        self.path = path
