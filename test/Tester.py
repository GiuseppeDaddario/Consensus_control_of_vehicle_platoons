import math

from src.Simulation import Simulation
from src.Viewer import Viewer


class Tester:
    def __init__(self, dt=0.01):
        self.dt = dt

    def run_simulation(self, sim: Simulation, render_every=10):
        viewer = Viewer()
        running = True
        step = 0

        while not sim.finished() and running:
            sim.step()

            if step % render_every == 0:
                running = viewer.handle_events()
                if not running:
                    break

                # draw
                viewer.draw_scene(
                    virtual_leader=sim.virtual_leader,
                    vehicles=sim.vehicles,
                    sim_time=sim.t,
                    virtual_leader_cmd=sim.last_virtual_leader_cmd,
                    histories=sim.log,
                    title=sim.title
                )
                viewer.tick(60)
            step += 1
        viewer.close()

    def test_gap_closing(self, render_every=10):
        P = [210.0, 140.0, 70.0, 0.0]
        V = [80 / 3.6] * 4
        A = [0.0] * 4

        sim = Simulation(dt=self.dt, mode="scenario", total_time=25.0,
                         P_init=P, V_init=V, A_init=A, scenario_name="Gap Closing")
        self.run_simulation(sim, render_every=render_every)

    def test_collision_avoidance(self, render_every=10):
        P = [81.66, 54.44, 27.22, 0.0]
        V = [80 / 3.6, 90 / 3.6, 100 / 3.6, 110 / 3.6]
        A = [0.0] * 4

        sim = Simulation(dt=self.dt, mode="scenario", total_time=15.0,
                         P_init=P, V_init=V, A_init=A, scenario_name="Collision Avoidance")
        self.run_simulation(sim, render_every=render_every)

    def test_vehicle_following(self, render_every=10):
        P = [27.0, 18.0, 9.0, 0.0]
        V = [0.0] * 4
        A = [0.0] * 4

        sim = Simulation(dt=self.dt, mode="scenario", total_time=40.0,
                         P_init=P, V_init=V, A_init=A,
                         leader_profile=lambda t: 1.75,
                         scenario_name="Vehicle Following")
        self.run_simulation(sim, render_every=render_every)

    def test_platoon_forming(self, render_every=10):
        P = [150.0, 100.0, 50.0, 0.0]
        V = [15.0, 20.0, 25.0, 30.0]
        A = [1.0, -6.0, 2.0, -3.0]

        sim = Simulation(dt=self.dt, mode="scenario", total_time=40.0,
                         P_init=P, V_init=V, A_init=A, scenario_name="Platoon Forming")
        sim.v_ref = 30.0
        self.run_simulation(sim, render_every=render_every)

    def test_braking_m10(self, render_every=10):
        M = 10
        v_start = 17.0
        v_target = 50.0 / 3.6
        h = 0.3
        r = 3.0
        L = 5.0
        d_eq = r + h * v_start + L
        P = [(M - 1 - i) * d_eq for i in range(M)]
        V = [v_start] * M
        A = [0.0] * M
        t_start_brake = 5.0
        T_brake = 1.5
        delta_v = abs(v_target - v_start)
        A_sine = (math.pi * delta_v) / (2.0 * T_brake)

        def braking_profile(t):
            if t < t_start_brake:
                return 0.0
            elif t <= t_start_brake + T_brake:
                return -A_sine * math.sin(math.pi * (t - t_start_brake) / T_brake)
            else:
                return 0.0

        sim = Simulation(dt=self.dt, mode="scenario", total_time=20.0,
                         P_init=P, V_init=V, A_init=A,
                         h=h, c1=0.8, c2=0.2,
                         leader_profile=braking_profile,
                         scenario_name="Braking (M=10)")

        sim.v_ref = v_start
        sim.p_ref = P[0]

        self.run_simulation(sim, render_every=render_every)




if __name__ == "__main__":
    tester = Tester(dt=0.001)
    tester.test_gap_closing(render_every=25)
    #tester.test_collision_avoidance(render_every=20)
    #tester.test_vehicle_following(render_every=30)
    #tester.test_platoon_forming(render_every=30)
    #tester.test_braking_m10(render_every=20)