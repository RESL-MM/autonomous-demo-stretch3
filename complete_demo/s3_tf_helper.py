from dataclasses import dataclass
import numpy as np

# TODO: check if the assumption that joint state 0 (for head nav and gripper at least) is
# in line with base forward...

# CAM_NAME HAS TO BE THE SAME AS THE JOINT STATE NAME?

def get

@dataclass
class CameraInfo:
    cam_name: str
    fixed_pos_offset: np.ndarray
    variable_offsets: dict[str, (str, float)] # dict of joint_name : (relevant axis, joint_value)

class CoordinateFrameHelper:
    def __init__(self, robot):
        self.robot = robot
        self.cameras: dict[str, CameraInfo] = {}

    def add_camera(self, camera: CameraInfo) -> None:
        self.cameras[camera.cam_name] = camera
    
    def get_camera(self, cam_name: str) -> CameraInfo:
        if cam_name not in self.cameras:
            raise KeyError(f"{cam_name} is not stored in the camera list...")
        return self.cameras[cam_name]
    
    def get_transform(self, cam_name) -> np.ndarray:
        # given the name of a camera in the stored cameras, return a 4x4 transform matrix
        # from camera space to world space coordinates (homogenous coordinates)
        
        T_fixed = np.eye(3)
