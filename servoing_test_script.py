#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from geometry_msgs.msg import TransformStamped
import argparse
import math
import hello_helpers.hello_misc as hm
from microfab_actions.action import MoveTo
from microfab_actions.action import PerformTask
from microfab_actions.action import TagSearch
from tag_search_helper import TagHelper
from gripper_cam_helper import GripperCamHelper
import numpy as np
import time
import yaml
from enum import Enum
from pathlib import Path



class AutonomousDemo(hm.HelloNode):
    def __init__(self, num_runs=1):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'autonomous_demo', 'autonomous_demo', wait_for_first_pointcloud=False)

        self.get_logger().info("initializing...")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("initializing TagHelper...")
        self.tag_helper = TagHelper(self.tf_buffer)
        self.get_logger().info("TagHelper initialized! Starting GripperCamHelper...")
        self.gripper_cam_helper = GripperCamHelper(GRIPPER_TAGS_DIR)
        self.get_logger().info("GripperCamHelper initialized!")

    def handle_search_state(self, target_tag_name):
        # simply search for and return the target tag's transform
        # try to query TagHelper, if not found do some correction and try again
        # try for a bit (e.g. 3 position adjustments) and report error if nothing
        
        # query priority for position: head nav THEN gripper; for manipulation: gripper THEN head nav
        # TODO: make a dictionary mapping target tag to official frame names (e.g. head name and gripper name)

        if self.previous_state == "MOVE_TO":
            self.tag_transforms[target_tag_name] = self.tag_helper.get_pose(target_tag_name, self.tag_reference[target_tag_name])
        elif self.previous_state == "MACHINE_OP" or self.previous_state == "WAFER_OP":
            gripper_tag_left = self.gripper_tag_names[target_tag_name]["left"]
            gripper_tag_right = self.gripper_tag_names[target_tag_name]["right"]
            head_nav_tag = self.head_nav_tag_names[target_tag_name]
            self.tag_transforms[gripper_tag_left] = self.tag_helper.get_pose(gripper_tag_left, self.tag_reference[gripper_tag_left])
            self.tag_transforms[gripper_tag_right] = self.tag_helper.get_pose(gripper_tag_right, self.tag_reference[gripper_tag_right])
            self.tag_transforms[head_nav_tag] = self.tag_helper.get_pose(head_nav_tag, self.tag_reference[head_nav_tag])

    # helper function for calculating the offset destination point based on the current tag and transform
    def get_destination_point(self, target_tag_name, target_tag_transform):

        pass

    # helper function for fine grain base movement (rotate THEN translate-- blocking)
    def move_base_rt(self, distance = 0.0, angle = 0.0):
        self.move_to_pose({'rotate_mobile_base': angle}, blocking=True)
        self.move_to_pose({'translate_mobile_base': distance}, blocking=True)

    # helper function for fine grain base movement (rotate THEN translate-- blocking)
    def move_base_tr(self, distance = 0.0, angle = 0.0):
        self.move_to_pose({'translate_mobile_base': distance}, blocking=True)
        self.move_to_pose({'rotate_mobile_base': angle}, blocking=True)
    
    # helper function for fine grain arm movement (blocking)
    def move_arm(self, up = 0.0, out = 0.0):
        # TODO: add pre-move stow and post-move restore?
        self.move_to_pose({'joint_arm': up}, blocking=True)
        self.move_to_pose({'joint_lift': out}, blocking=True)
        
    def rotate_wrist(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        self.move_to_pose({'joint_wrist_roll': roll,
                           'joint_wrist_pitch': pitch,
                           'joint_wrist_yaw': yaw},
                           blocking=True)

    def stow_gripper(self):
        self.move_to_pose({'gripper_aperture': 0.03,
                           'joint_wrist_pitch': 0.0,
                           'joint_wrist_roll': 0.0,
                           'joint_wrist_yaw': 3*math.pi/4,
                           'joint_arm': 0.0,
                           'joint_lift': 0.2},
                             blocking=True)

    def load_config_values(self, file_path):
        with open(file_path, 'r') as config_file:
            return yaml.safe_load(config_file)

def main(args=None):
    # TODO: implement argparser to set number of iterations
    try:
        demo_node = AutonomousDemo()
        # demo_node.autonomous_demo_sm_loop()
    except Exception as e:
            demo_node.get_logger().info('Interrupt received, so shutting down')
            demo_node.get_logger().info(e)
            demo_node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()