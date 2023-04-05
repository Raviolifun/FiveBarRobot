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
    def __init__(self, bottom_length, mass_matrix, length_matrix, a_length_matrix, q4_pos, i_matrix=None):
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

        self.q4_pos = q4_pos

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

    def get_q3_q4(self, q1, q2, qd1=0, qd2=0):

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

        if feasibility < 0:
            self.q4_pos = not self.q4_pos
            print("Entered singularity, recoverable?")
            feasibility = 0

        if self.q4_pos:
            # plus
            param = math.sqrt(feasibility)
        else:
            # minus
            param = -math.sqrt(feasibility)

        q4 = math.atan2(param, c_q1q2) + math.atan2(b_q1q2, a_q1q2) - q2

        # using this value of q4, find q3
        q3 = math.atan2(u_q1q2 + self.a4 * math.sin(q2 + q4), y_q1q2 + self.a4 * math.cos(q2 + q4)) - q1

        return q3, q4

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

        # dirt basic controller, I give it desired positions manually from here
        goal_q1 = math.radians(180)
        goal_q2 = math.radians(0)

        error_q1 = goal_q1 - q1
        error_q2 = goal_q2 - q2

        error_q1 = error_q1 % (2 * math.pi)
        if abs(error_q1) > math.pi:
            error_q1 = -error_q1

        error_q2 = error_q2 % (2 * math.pi)
        if abs(error_q2) > math.pi:
            error_q2 = -error_q2

        t1 = error_q1 * 10 - qd1 * 1
        t2 = error_q2 * 10 - qd2 * 1

        max_torque = 5

        if t1 > max_torque:
            t1 = max_torque
        elif t1 < -max_torque:
            t1 = -max_torque
        if t2 > max_torque:
            t2 = max_torque
        elif t2 < -max_torque:
            t2 = -max_torque

        print((t1, t2))

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
