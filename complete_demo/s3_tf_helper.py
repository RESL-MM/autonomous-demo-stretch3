from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation

# TODO: check if the assumption that joint state 0 (for head nav and gripper at least) is
# in line with base forward...

# CAM_NAME HAS TO BE THE SAME AS THE JOINT STATE NAME?

def make_homogenous_rt_tf_matrix(x: float, y: float, z: float,
                                 roll: float, pitch: float, yaw: float) -> np.ndarray:
    # angles in Rad
    R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T

def make_homogenous_r_tf_matrix(roll:float, pitch: float, yaw: float):
    # angles in Rad
    return make_homogenous_rt_tf_matrix(0, 0, 0, roll, pitch, yaw)


@dataclass
class CameraInfo:
    # TODO: check assumption that there is no need for fixed rotation frame-- i.e. both camera
    # are... "naturally" parallel to the base frame...? If not, store transform matrix instead
    cam_name: str
    fixed_pos_offset: np.ndarray

class TransformHelper:
    def __init__(self, robot):
        self.robot = robot
        self.cameras: dict[str, CameraInfo] = {}

    def add_camera(self, camera: CameraInfo) -> None:
        self.cameras[camera.cam_name] = camera
    
    def get_camera(self, cam_name: str) -> CameraInfo:
        if cam_name not in self.cameras:
            raise KeyError(f"{cam_name} is not stored in the camera list...")
        return self.cameras[cam_name]
    

    def get_T_base_cam(self, cam_name,
                          x: float, y: float, z: float,
                          roll: float, pitch: float, yaw: float) -> np.ndarray:
        # returns the transform matrix FROM cam TO base frame
        cam_offset = self.cameras[cam_name].fixed_pos_offset.copy()
        cam_offset[:3] += [x, y, z]

        return make_homogenous_rt_tf_matrix(cam_offset[0], cam_offset[1], cam_offset[2], roll, pitch, yaw)
    
    def get_base_coord_from_cam_coord(self, cam_name,
                                      px: float, py: float, pz: float,
                                      x: float, y: float, z: float,
                                      roll: float, pitch: float, yaw: float) -> np.ndarray:
        
        cam_point_homogenous = np.array([px, py, pz, 1.0])

        Twc = self.get_T_base_cam(cam_name, x, y, z, roll, pitch, yaw)

        return Twc @ cam_point_homogenous