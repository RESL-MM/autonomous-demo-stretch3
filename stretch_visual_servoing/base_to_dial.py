import numpy as np
import d405_helpers as dh
import pyrealsense2 as rs
import numpy as np
import cv2
import normalized_velocity_control as nvc
import stretch_body.robot as rb
import time
import aruco_detector as ad
import aruco_to_fingertips as af
import yaml
from yaml.loader import SafeLoader
from scipy.spatial.transform import Rotation
from hello_helpers import hello_misc as hm
import argparse
import loop_timer as lt
from stretch_body import robot_params
from stretch_body import hello_utils as hu

####################################
## Initial Pose

joint_state_center = {
    'lift_pos' : 0.7,
    'arm_pos': 0.01,
    'wrist_yaw_pos': 0.0,
    'wrist_pitch_pos': -0.4, #-0.6
    'wrist_roll_pos': 0.0,
    'gripper_pos': 0
}

movement = {
    'stage1_forward' : 1.11,
    'stage1_rotate' : -np.pi/2,
    'stage2_forward' : 2.0,
    'stage3_forward' : 0.45,
}

def recenter_robot(robot):
    pan = np.pi/2.0
    tilt = -np.pi/2.0
    robot.head.move_to('head_pan', pan)
    robot.head.move_to('head_tilt', tilt)
    robot.push_command()
    robot.wait_command()


    robot.arm.move_to(joint_state_center['arm_pos'])
    robot.push_command()
    robot.wait_command()
    
    robot.end_of_arm.get_joint('wrist_yaw').move_to(joint_state_center['wrist_yaw_pos'])
    robot.end_of_arm.get_joint('wrist_pitch').move_to(joint_state_center['wrist_pitch_pos'])
    robot.push_command()
    robot.wait_command()
    
    robot.end_of_arm.get_joint('wrist_roll').move_to(joint_state_center['wrist_roll_pos'])
    robot.push_command()
    robot.wait_command()
    
    robot.lift.move_to(joint_state_center['lift_pos'])
    robot.push_command()
    robot.wait_command()
    
def main():
    robot = rb.Robot()
    robot.startup()
    recenter_robot(robot)

    # Move forward 1 meter
    robot.base.translate_by(movement['stage1_forward'], v_m=0.3)
    robot.push_command()
    robot.wait_command()
    
    # Rotate right by 45 degrees
    robot.base.rotate_by(movement['stage1_rotate'])
    robot.push_command()
    robot.wait_command()
    
    # Move forward 2 meter 
    robot.base.translate_by(movement['stage2_forward'], v_m=0.3)
    robot.push_command()
    robot.wait_command()
    
    robot.base.translate_by(movement['stage3_forward'], v_m=0.3)
    robot.push_command()
    robot.wait_command()
    
    time.sleep(10.0)
    
    # reverse the movement  
    robot.base.translate_by(-movement['stage3_forward'], v_m=0.3)
    robot.push_command()
    robot.wait_command()
    
    robot.base.translate_by(-movement['stage2_forward'], v_m=0.3)
    robot.push_command()
    robot.wait_command()
    
    robot.base.rotate_by(-movement['stage1_rotate'])
    robot.push_command()
    robot.wait_command()
    
    robot.base.translate_by(-movement['stage1_forward'], v_m=0.3)
    robot.push_command()
    robot.wait_command()
    
    robot.stop()

if __name__ == '__main__':
    main()
