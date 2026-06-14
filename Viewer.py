import rerun as rr
from scipy.spatial.transform import Rotation as R


class Viewer:
    def __init__(self, spawn=True):
        rr.init("Bidirectional_Platoon", spawn=spawn)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        self.car_width = 2.0
        self.car_height = 1.5
        self.meshes_loaded = set()

        rot_quat = R.from_euler('zyx', [0, 0, 90], degrees=True).as_quat()
        rr.log(
            "world/ground",
            rr.Asset3D(path="assets/map.glb"),
            rr.Transform3D(
                translation=[150.0, 20.0, 10.7],
                rotation=rr.Quaternion(xyzw=rot_quat),
                scale=22.0
            ),
            static=True
        )

    def set_time(self, time_sec):
        rr.set_time("sim_time", duration=time_sec)

    def draw_vehicle(self, vehicle, is_leader=False):
        entity_path = "world/platoon/Virtual_Leader" if is_leader else f"world/platoon/Car_{vehicle.id}"

        if vehicle.id not in self.meshes_loaded:
            if vehicle.mesh_path:
                rot_quat = R.from_euler('zyx', [vehicle.yaw, vehicle.pitch, vehicle.roll], degrees=True).as_quat()
                rr.log(
                    f"{entity_path}/mesh",
                    rr.Asset3D(path=vehicle.mesh_path),
                    rr.Transform3D(rotation=rr.Quaternion(xyzw=rot_quat), scale=vehicle.scale),
                    static=True
                )
            else:
                half_sizes = [[vehicle.length / 2, self.car_width / 2, self.car_height / 2]]
                center = [[0.0, 0.0, self.car_height / 2]]
                rr.log(f"{entity_path}/body",
                       rr.Boxes3D(half_sizes=half_sizes, centers=center, colors=[[150, 150, 150]]), static=True)

            self.meshes_loaded.add(vehicle.id)

        rr.log(entity_path, rr.Transform3D(translation=[vehicle.p, 0.0, 0.0]))

        # CAMERA TRACKING #TODO
        if is_leader:
            cam_offset = [vehicle.p - 15.0, -10.0, 5.0]
            cam_quat = R.from_euler('zyx', [30, -15, 0], degrees=True).as_quat()
            rr.log("world/tracking_camera", rr.Transform3D(
                translation=cam_offset,
                rotation=rr.Quaternion(xyzw=cam_quat)
            ))
            rr.log("world/tracking_camera", rr.Pinhole(focal_length=500, resolution=[1920, 1080]))

    def log_telemetry(self, vehicle):
        rr.log(f"telemetry/car_{vehicle.id}/velocity", rr.Scalars(vehicle.v))
        rr.log(f"telemetry/car_{vehicle.id}/command_u", rr.Scalars(vehicle.u))