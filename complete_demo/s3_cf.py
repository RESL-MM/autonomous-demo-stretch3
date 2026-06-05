from dataclass import dataclass
from typing import Callable, Optional
import numpy as np
from scipy.spatial.transform import Rotation as R

def make_transform(rotation: np.ndarray, translation: np.ndarray)->np.ndarray:
    # given rotation and translation from camera to world/stretch3-base coordinates,
    # return a 4x4 transform matrix
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(translation).reshape(3)
    
    return T

def invert_transform(T: np.ndarray) -> np.ndarray:
    Rm = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = Rm.T
    T_inv[:3, 3] = -Rm.T @ T

    return T_inv

@dataclass
class CameraState:
    cam_name: str
    fixed_offset: np.ndarray

class CoordinateFrameHelper:
    def __init__(self):
        self.cameras: dict[str, CameraState] = {}

    def add_camera(self, camera: CameraState) -> None:
        self.cameras[camera.cam_name] = camera

    def get_camera(self, cam_name: str) -> CameraState:
        if cam_name not in self.cameras:
            raise KeyError(f"{cam_name} is not in the stored list of cameras...")
        return self.cameras[cam_name]
    
    # TODO: finish implementing coordinate helper class (actually write transformation code and stuff...)