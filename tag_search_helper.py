#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import numpy as np
import time

# helper class used by autonomous demo that helps retrieve tag transforms
# and handle issues (e.g tag not in sight)

class TagHelper:
    def __init__(self, tf_buffer, max_tag_valid_time=0.1):
        self.tf_buffer = tf_buffer
        self.max_tag_valid_time = max_tag_valid_time
        self.tag_last_poses = {}

    def get_pose(self, target_tag_frame, reference_frame="base_link"):
        # attempt to read tf tree and return the transform
        try:
            tag_transform = self.tf_buffer.lookup_transform(reference_frame,
                                                            target_tag_frame,
                                                            rclpy.time.Time())
            self.tag_last_poses[target_tag_frame] = (tag_transform, time.time())
            return tag_transform
        except Exception:
            # attempt to return last known pose if valid
            if target_tag_frame in self.tag_last_poses:
                tag_transform, pose_time = self.tag_last_poses[target_tag_frame]
                if time.time()-pose_time < self.max_tag_valid_time:
                    return tag_transform
            return None