import pygame
import numpy as np
from src.Controller import BidirectionalController
from src.Vehicle import Vehicle


class Simulation:
    def __init__(self, dt=0.01, mode="interactive", total_time=60.0, P_init=None, V_init=None, A_init=None, h=1.0, c1=0.8, c2=0.2, virtual_leader_profile=None, virtual_leader_max_v=40.0, scenario_name=None):
        self.dt = dt
        self.title = "Manual mode" if scenario_name is None else scenario_name
        self.mode = mode
        self.total_time = total_time
        self.virtual_leader_profile = virtual_leader_profile
        self.virtual_leader_max_v = virtual_leader_max_v

        if P_init is None:
            P_init = [0.0, -20.0, -40.0, -60.0, -80.0]
        if V_init is None:
            V_init = [15.0] * len(P_init)
        if A_init is None:
            A_init = [0.0] * len(P_init)

        self.num_vehicles = len(P_init) - 1
        self.controller = BidirectionalController(self.num_vehicles, h=h, c1=c1, c2=c2)

        self.p_ref = P_init[0]
        self.v_ref = V_init[0]
        self.a_ref = A_init[0]

        self.virtual_leader = Vehicle(id="virtual leader", start_pos=P_init[0], start_vel=V_init[0], length=5.0)
        self.virtual_leader.a = A_init[0]

        self.vehicles = []
        for i in range(self.num_vehicles):
            veh = Vehicle(id=i, start_pos=P_init[i + 1], start_vel=V_init[i + 1], length=5.0)
            veh.a = A_init[i + 1]
            self.vehicles.append(veh)

        self.t = 0.0
        self.last_virtual_leader_cmd = 0.0

        self.log = {
            "t": [],
            "v": [[] for _ in range(self.num_vehicles + 1)],
            "a": [[] for _ in range(self.num_vehicles + 1)],
            "u": [[] for _ in range(self.num_vehicles + 1)],
            "d": [[] for _ in range(self.num_vehicles)],
            "e": [[] for _ in range(self.num_vehicles)],
        }

    def finished(self):
        return self.t >= self.total_time

    def _interactive_command(self):
        keys = pygame.key.get_pressed()
        raw = 0.0
        if keys[pygame.K_w]:
            raw += 2.0
        if keys[pygame.K_s]:
            raw -= 6.0
        return raw

    def _scenario_command(self):
        if self.virtual_leader_profile is None:
            self.a_ref = 0.0
        else:
            if self.v_ref < self.virtual_leader_max_v:
                self.a_ref = self.virtual_leader_profile(self.t)
            else:
                self.a_ref = 0.0

        self.v_ref += self.a_ref * self.dt
        self.p_ref += self.v_ref * self.dt
        r_star = np.array([self.p_ref, self.v_ref, self.a_ref])

        return self.controller.compute_virtual_leader_u(r_star, self.virtual_leader.get_state())

    def step(self):
        if self.mode == "interactive":
            nominal_leader_u = self._interactive_command()
        else:
            nominal_leader_u = self._scenario_command()

        safe_virtual_leader_cmd = self.controller.apply_cbf(nominal_leader_u, self.virtual_leader)
        self.virtual_leader.step(self.dt, safe_u=safe_virtual_leader_cmd)

        u_dots = self.controller.compute_nominal_commands(self.vehicles, self.virtual_leader)

        for i in range(self.num_vehicles):
            curr_veh = self.vehicles[i]

            nominal_u = curr_veh.u + u_dots[i] * self.dt
            safe_u = self.controller.apply_cbf(nominal_u, curr_veh)
            curr_veh.step(self.dt, safe_u)

        self.last_virtual_leader_cmd = safe_virtual_leader_cmd
        self._log()
        self.t += self.dt

    def _log(self):
        self.log["t"].append(self.t)

        self.log["v"][0].append(self.virtual_leader.v)
        self.log["a"][0].append(self.virtual_leader.a)
        self.log["u"][0].append(self.virtual_leader.u)

        for i, veh in enumerate(self.vehicles):
            self.log["v"][i + 1].append(veh.v)
            self.log["a"][i + 1].append(veh.a)
            self.log["u"][i + 1].append(veh.u)

            prev_veh = self.virtual_leader if i == 0 else self.vehicles[i - 1]
            self.log["d"][i].append(prev_veh.p - veh.p - veh.length)

            next_veh = self.vehicles[i + 1] if i < self.num_vehicles - 1 else None
            e_val = self.controller.compute_errors(veh, prev_veh, next_veh)[0]
            self.log["e"][i].append(e_val)