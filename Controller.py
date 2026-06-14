import numpy as np
from scipy.signal import place_poles


class BidirectionalController:
    def __init__(self, num_vehicles, h=1.0, c1=0.8, c2=0.2, tau=0.1):
        self.M = num_vehicles
        self.h = h
        self.c1 = c1
        self.c2 = c2
        self.tau = tau
        self.r = 5.0

        self.K = np.array([2.0, 1.0, 0.0])

        # error dynamics
        self.A = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, -1.0 / self.tau]
        ])
        self.B = np.array([
            [0],
            [0],
            [1.0 / self.tau]
        ])

        desired_poles = np.array([-1.0, -2.0, -3.0])
        self.K_bar = place_poles(self.A, self.B, desired_poles).gain_matrix.flatten()

        # CBF Parameters
        self.a_max = 2.0
        self.a_min = -6.0
        self.u_max = 2.0
        self.u_min = -6.0
        self.v_max = 40.0
        self.v_min = 0.0

        self.b1_a_max = 5.0
        self.b1_a_min = 15.0
        self.B_v_max = np.array([1.0, 2.0])  # roots at epsilon = -1
        self.B_v_min = np.array([1.0, 2.0])  # roots at epsilon = -1

    def compute_virtual_leader_u(self, r_star, x_0):
        u_0 = np.dot(self.K_bar, (r_star - x_0))
        return u_0

    def compute_errors(self, curr_veh, prev_veh, next_veh):
        e_f = prev_veh.p - curr_veh.p - curr_veh.length - (self.r + self.h * curr_veh.v)
        e_f_dot = prev_veh.v - curr_veh.v - self.h * curr_veh.a
        e_f_ddot = prev_veh.a - curr_veh.a - self.h * ((curr_veh.u - curr_veh.a) / curr_veh.tau)

        if next_veh is not None:
            e_b = curr_veh.p - next_veh.p - curr_veh.length - (self.r + self.h * next_veh.v)
            e_b_dot = curr_veh.v - next_veh.v - self.h * next_veh.a
            e_b_ddot = curr_veh.a - next_veh.a - self.h * ((next_veh.u - next_veh.a) / next_veh.tau)
        else:
            e_b = 0.0
            e_b_dot = 0.0
            e_b_ddot = 0.0

        e_1 = self.c1 * e_f - self.c2 * e_b
        e_2 = self.c1 * e_f_dot - self.c2 * e_b_dot
        e_3 = self.c1 * e_f_ddot - self.c2 * e_b_ddot

        return np.array([e_1, e_2, e_3])

    def compute_nominal_commands(self, vehicles, virtual_leader):
        u_dots = np.zeros(self.M)
        x_errs = []

        # Precompute all state errors (x_i) for the entire platoon
        for i in range(self.M):
            curr_veh = vehicles[i]
            prev_veh = virtual_leader if i == 0 else vehicles[i - 1]
            next_veh = vehicles[i + 1] if i < self.M - 1 else None
            x_errs.append(self.compute_errors(curr_veh, prev_veh, next_veh))

        # Compute zeta and u_dots backwards
        for i in range(self.M - 1, -1, -1):
            curr_veh = vehicles[i]
            prev_veh = virtual_leader if i == 0 else vehicles[i - 1]
            next_veh = vehicles[i + 1] if i < self.M - 1 else None

            x_i = x_errs[i]

            if i == 0:
                # Leader: communicates with follower plus g_ii = 1
                zeta_i = np.dot(self.K, x_errs[1] - x_i) - np.dot(self.K, x_i)
            elif i == self.M - 1:
                # Last vehicle: communicates only with predecessor
                zeta_i = np.dot(self.K, x_errs[i - 1] - x_i)
            else:
                # Middle vehicles: bidirectional communication
                zeta_i = np.dot(self.K, x_errs[i - 1] - x_i) + np.dot(self.K, x_errs[i + 1] - x_i)

            # Last vehicle dynamics
            if i == self.M - 1:
                u_dots[i] = -(1.0 / self.h) * curr_veh.u \
                            - (1.0 / (self.h * self.c1)) * zeta_i \
                            + (1.0 / self.h) * prev_veh.u

            # Standard vehicle dynamics
            else:
                u_dots[i] = -(1.0 / (self.h * self.c1)) * curr_veh.u \
                            + (1.0 / self.h) * prev_veh.u \
                            - (1.0 / (self.h * self.c1)) * zeta_i \
                            + (self.c2 / (self.h * self.c1)) * next_veh.u \
                            + (self.c2 / self.c1) * u_dots[i + 1]

        return u_dots

    def apply_cbf(self, nominal_u, curr_veh, prev_veh):
        v = curr_veh.v
        a = curr_veh.a
        tau = curr_veh.tau

        # actuator constraints
        upper_u = self.u_max
        lower_u = self.u_min

        # acceleration constraints
        upper_a = a + tau * self.b1_a_max * (self.a_max - a)
        lower_a = a - tau * self.b1_a_min * (a - self.a_min)

        # velocity constraints
        upper_v = a + tau * (self.B_v_max[0] * (self.v_max - v) - self.B_v_max[1] * a)
        lower_v = a - tau * (self.B_v_min[0] * (v - self.v_min) + self.B_v_min[1] * a)

        # QP Solver
        max_bound = min(upper_u, upper_a, upper_v)
        min_bound = max(lower_u, lower_a, lower_v)

        # clip inside safe bounds
        if min_bound > max_bound:
            safe_u = min_bound
        else:
            safe_u = np.clip(nominal_u, min_bound, max_bound)

        return safe_u