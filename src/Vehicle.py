import numpy as np
import trimesh

class Vehicle:
    def __init__(self, id, start_pos, start_vel, tau=0.1, length=5.0 ):
        self.id = id
        self.p = start_pos
        self.v = start_vel
        self.a = 0.0
        self.u = 0.0
        self.tau = tau
        self.length = length

    def get_state(self):
        return np.array([self.p, self.v, self.a])

    def step(self, dt, safe_u):
        self.u = safe_u
        a_dot = (1.0 / self.tau) * (self.u - self.a)

        self.p += self.v * dt
        self.v += self.a * dt
        self.a += a_dot * dt