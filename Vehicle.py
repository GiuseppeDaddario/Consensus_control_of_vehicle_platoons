import numpy as np
import trimesh

class Vehicle:
    def __init__(self, id, start_pos, start_vel, tau=0.1, mesh_path=None, scale=1.0, yaw=0, pitch=0, roll=0):
        self.id = id
        self.p = start_pos
        self.v = start_vel
        self.a = 0.0
        self.u = 0.0
        self.tau = tau
        self.mesh_path = mesh_path
        self.scale = scale
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

        # Auto-calculate length from 3D mesh
        if self.mesh_path:
            mesh = trimesh.load(self.mesh_path, force='mesh')
            # Assuming X is the forward axis. Multiply by scale to get real-world meters.
            self.length = mesh.bounding_box.extents[0] * self.scale
        else:
            self.length = 4.5 # Fallback

    def get_state(self):
        return np.array([self.p, self.v, self.a])

    def step(self, dt, safe_u):
        self.u = safe_u
        a_dot = (1.0 / self.tau) * (self.u - self.a)

        self.p += self.v * dt
        self.v += self.a * dt
        self.a += a_dot * dt