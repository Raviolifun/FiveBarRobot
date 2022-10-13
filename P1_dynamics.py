import math
import numpy as np


class Dynamics:
    def __init__(self, mass_matrix, length_matrix):
        self.MA = mass_matrix[0]
        self.MB = mass_matrix[1]
        self.MC = mass_matrix[2]
        self.MD = mass_matrix[3]

        self.L0 = length_matrix[0]
        self.L1 = length_matrix[1]
        self.L2 = length_matrix[2]
        self.L3 = length_matrix[3]
        self.L4 = length_matrix[4]

        self.g = 9.81

    def f(self, t, y):
        q1, q2, q3, q4, qd1, qd2, qd3, qd4, T1, T4 = y

        q = np.array([[q1], [q2], [q3], [q4]])
        qd = np.array([[qd1], [qd2], [qd3], [qd4]])

        T1 = 0
        T4 = 1

        # ==================================================================================
        # ==============================      Dynamics      ================================
        # ==================================================================================

        C12 = math.cos(q1 - q2)
        C34 = math.cos(q3 - q4)
        S12 = math.sin(q1 - q2)
        S34 = math.sin(q3 - q4)

        M1 = np.array([(self.MA/3+self.MB) * self.L1 * self.L1, self.MB * self.L1 * self.L2 / 4 * C12, 0, 0])
        M2 = np.array([self.MB * self.L1 * self.L2 / 4 * C12, self.MB * self.L2 * self.L2 / 3, 0, 0])
        M3 = np.array([0, 0, self.MC * self.L3 * self.L3 / 3, self.MC * self.L3 * self.L4 / 4 * C34])
        M4 = np.array([0, 0, self.MC * self.L3 * self.L4 / 4 * C34, (self.MD/3 + self.MC) * self.L4 * self.L4])

        M = np.array([M1, M2, M3, M4])

        inertia_1 = self.MB * self.L1 * self.L2 / 4
        inertia_2 = self.MC * self.L3 * self.L4 / 4

        C1 = np.array([inertia_1 * qd2 * S12, -inertia_1 * (qd1 - qd2) * S12, 0, 0])
        C2 = np.array([-inertia_1 * (qd1 - qd2) * S12, -inertia_1 * qd1 * S12, 0, 0])
        C3 = np.array([0, 0, inertia_2 * qd4 * S34, -inertia_2 * (qd3 - qd4) * S34])
        C4 = np.array([0, 0, -inertia_2 * (qd3 - qd4) * S34, inertia_2 * qd3 * S34])

        C = np.array([C1, C2, C3, C4])

        T = np.array([[T1], [0], [0], [T4]])

        Th = np.array([[0], [0], [0], [0]])

        right_side = (T + Th - np.matmul(C, qd))
        left_side = np.linalg.inv(M)

        qdd = np.matmul(left_side, right_side)

        qdd1, qdd2, qdd3, qdd4 = qdd

        # ==================================================================================
        # ===========================     Controller time     ==============================
        # ==================================================================================

        TD1 = 0
        TD4 = 0

        return [qd1, qd2, qd3, qd4, qdd1, qdd2, qdd3, qdd4, TD1, TD4]
