from Controller import BidirectionalController
from Vehicle import Vehicle
from Viewer import Viewer

def main():
    dt = 0.005
    total_time = 30.0

    # Leader
    leader = Vehicle(
        id="Leader", start_pos=0.0, start_vel=15.0,
        mesh_path="assets/police.glb", scale=100.0, yaw= 90, pitch=90
    )

    # Followers
    specs = [
        {'mesh': 'assets/bike.glb', 'scale': 2.0, 'yaw': 0, 'pitch': 90, 'roll': 90},
        {'mesh': 'assets/mclaren.glb', 'scale': 0.8, 'yaw': 0, 'pitch': 90, 'roll': 90},
        {'mesh': 'assets/mercedes.glb', 'scale': 1.5, 'yaw': 0, 'pitch': 90, 'roll': 90},
        {'mesh': 'assets/truck.glb', 'scale': 0.01, 'yaw': 0, 'pitch': 90, 'roll': 90}
    ]

    viewer = Viewer()
    num_vehicles = len(specs)
    controller = BidirectionalController(num_vehicles)

    vehicles = []
    for i in range(num_vehicles):
        vehicles.append(Vehicle(
            id=i,
            start_pos=-(i + 1) * 20.0,
            start_vel=15.0,
            mesh_path=specs[i]['mesh'],
            scale=specs[i]['scale'],
            yaw=specs[i]['yaw'],
            pitch=specs[i]['pitch'],
            roll=specs[i]['roll']
        ))

    for step in range(int(total_time / dt)):
        time_sec = step * dt
        viewer.set_time(time_sec)

        leader_cmd = 0.0
        if 2.0 < time_sec < 5.0:
            leader_cmd = -7.0
        elif 6.0 < time_sec < 9.0:
            leader_cmd = 3.0

        safe_leader_cmd = controller.apply_cbf(leader_cmd, leader, None)
        leader.step(dt, safe_u=safe_leader_cmd)

        u_dots = controller.compute_nominal_commands(vehicles, leader)

        for i in range(num_vehicles):
            curr_veh = vehicles[i]
            prev_veh = leader if i == 0 else vehicles[i - 1]

            nominal_u = curr_veh.u + u_dots[i] * dt
            safe_u = controller.apply_cbf(nominal_u, curr_veh, prev_veh)
            curr_veh.step(dt, safe_u)

            viewer.draw_vehicle(curr_veh, is_leader=False)
            viewer.log_telemetry(curr_veh)

        viewer.draw_vehicle(leader, is_leader=True)

if __name__ == "__main__":
    main()