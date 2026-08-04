import d405_helpers as dh
import aruco_detector as ad
import pyrealsense2 as rs
import numpy as np
import cv2
import yaml
from yaml.loader import SafeLoader
from scipy.spatial.transform import Rotation # need this?

class GripperCamHelper:
    def __init__(self, gripper_tags_config, exposure='low', debug=False):
        with open(gripper_tags_config) as f:
            gripper_tag_info = yaml.load(f, Loader=SafeLoader)

        self.aruco_detector = ad.ArucoDetector(marker_info=gripper_tag_info, show_debug_images=debug, brighten_images=True)

        self.pipeline, self.profile = dh.start_d405(exposure)
        self.camera_info = None
        self.first_frame = True

    def update(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        
        if self.first_frame:
            self.camera_info = dh.get_camera_info(color_frame)
            self.first_frame = False

        image = np.asanyarray(color_frame.get_data())
        self.aruco_detector.update(image, self.camera_info)

        return self.aruco_detector.get_detected_marker_dict()
    
    def shutdown(self):
        self.pipeline.stop()