import pygame
from src.Controller import BidirectionalController
from src.Vehicle import Vehicle
from src.Viewer import Viewer


def append_histories(histories, controller, leader, vehicles, t):
    histories["t"].append(t)

    histories["v"][0].append(leader.v)
    histories["a"][0].append(leader.a)
    histories["u"][0].append(leader.u)

    for i, veh in enumerate(vehicles):
        histories["v"][i + 1].append(veh.v)
        histories["a"][i + 1].append(veh.a)
        histories["u"][i + 1].append(veh.u)

        prev_veh = leader if i == 0 else vehicles[i - 1]
        histories["d"][i].append(prev_veh.p - veh.p - veh.length)

        next_veh = vehicles[i + 1] if i < len(vehicles) - 1 else None
        e_val = controller.compute_errors(veh, prev_veh, next_veh)[0]
        histories["e"][i].append(e_val)


def main():
    dt = 0.005
    total_time = 300.0

    store_every = 5
    render_every = 5

    leader = Vehicle(id="Leader", start_pos=0.0, start_vel=15.0, length=5.0)

    num_vehicles = 4
    controller = BidirectionalController(num_vehicles)

    vehicles = []
    for i in range(num_vehicles):
        vehicles.append(
            Vehicle(
                id=i,
                start_pos=-(i + 1) * 20.0,
                start_vel=15.0,
                length=5.0
            )
        )

    histories = {
        "t": [],
        "v": [[] for _ in range(num_vehicles + 1)],
        "a": [[] for _ in range(num_vehicles + 1)],
        "u": [[] for _ in range(num_vehicles + 1)],
        "d": [[] for _ in range(num_vehicles)],
        "e": [[] for _ in range(num_vehicles)],
    }

    viewer = Viewer()
    running = True
    step = 0
    sim_time = 0.0

    while running and sim_time < total_time:
        if step % render_every == 0:
            running = viewer.handle_events()
            if not running:
                break

        keys = pygame.key.get_pressed()

        raw_leader_cmd = 0.0
        if keys[pygame.K_w]:
            raw_leader_cmd += 2.0
        if keys[pygame.K_s]:
            raw_leader_cmd -= 6.0

        safe_leader_cmd = controller.apply_cbf(raw_leader_cmd, leader, None)
        leader.step(dt, safe_u=safe_leader_cmd)

        u_dots = controller.compute_nominal_commands(vehicles, leader)

        for i in range(num_vehicles):
            curr_veh = vehicles[i]
            prev_veh = leader if i == 0 else vehicles[i - 1]

            nominal_u = curr_veh.u + u_dots[i] * dt
            safe_u = controller.apply_cbf(nominal_u, curr_veh, prev_veh)
            curr_veh.step(dt, safe_u)

        if step % store_every == 0:
            append_histories(histories, controller, leader, vehicles, sim_time)

        if step % render_every == 0:
            viewer.draw_scene(
                leader=leader,
                vehicles=vehicles,
                sim_time=sim_time,
                leader_cmd=safe_leader_cmd,
                histories=histories
            )
            viewer.tick(60)

        sim_time += dt
        step += 1

    viewer.close()


if __name__ == "__main__":
    main()