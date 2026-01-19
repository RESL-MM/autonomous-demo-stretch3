#!/usr/bin/env python3
"""
Head Camera-based Machine Operation Demo for Stretch 3
========================================================

This script demonstrates autonomous button pressing and switch flipping using:
- Head navigation camera (D435i) instead of gripper camera (D405)
- ArUco markers for button and switch localization
- Base-forward approach (robot faces the machine) instead of perpendicular grasp
- Forward kinematics for end effector position estimation

Author: Generated for Stretch 3 Research
Date: 2025
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import stretch_body.robot as rb
import time
import argparse
from scipy.spatial.transform import Rotation
from hello_helpers import hello_misc as hm
import yaml
from yaml.loader import SafeLoader

# Import existing modules
import aruco_detector as ad
import normalized_velocity_control as nvc
import loop_timer as lt

####################################
# HEAD CAMERA HELPERS
####################################

def start_head_camera(robot, exposure='auto'):
    """Initialize the D435i head navigation camera

    Args:
        robot: Stretch robot instance
        exposure: 'auto', 'low', 'medium', or integer value

    Returns:
        pipeline, profile, camera_info, depth_scale
    """
    # Find D435i camera (head camera)
    camera_info_list = [{'name': device.get_info(rs.camera_info.name),
                         'serial_number': device.get_info(rs.camera_info.serial_number)}
                        for device in rs.context().devices]

    print('All cameras found:')
    for info in camera_info_list:
        print(f"  - {info['name']}: {info['serial_number']}")
    print()

    # Find D435i (head camera)
    d435i_info = None
    for info in camera_info_list:
        if 'D435' in info['name'] and 'D405' not in info['name']:
            d435i_info = info
            break

    if d435i_info is None:
        print('WARNING: D435i head camera not found, trying to use any D435...')
        for info in camera_info_list:
            if 'D435' in info['name']:
                d435i_info = info
                break

    if d435i_info is None:
        raise RuntimeError('Head camera (D435i) not found!')

    print(f'Using head camera: {d435i_info["name"]} ({d435i_info["serial_number"]})')

    # Configure pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(d435i_info['serial_number'])

    # Use 640x480 @ 30fps for good performance
    width, height, fps = 640, 480, 30
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    profile = pipeline.start(config)

    # Set exposure
    if exposure == 'auto':
        sensor = profile.get_device().query_sensors()[1]  # RGB sensor
        sensor.set_option(rs.option.enable_auto_exposure, True)
    else:
        exposure_value = {'low': 11000, 'medium': 30000}.get(exposure, int(exposure))
        sensor = profile.get_device().query_sensors()[1]
        sensor.set_option(rs.option.exposure, exposure_value)

    # Get depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Get camera intrinsics
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

    camera_matrix = np.array([[intrinsics.fx, 0.0, intrinsics.ppx],
                              [0.0, intrinsics.fy, intrinsics.ppy],
                              [0.0, 0.0, 1.0]])

    distortion_coefficients = np.array(intrinsics.coeffs)

    camera_info = {
        'camera_matrix': camera_matrix,
        'distortion_coefficients': distortion_coefficients,
        'distortion_model': intrinsics.model
    }

    print(f'Head camera initialized: {width}x{height} @ {fps}fps')
    print(f'Depth scale: {depth_scale}')
    print()

    return pipeline, profile, camera_info, depth_scale


def get_head_camera_to_base_transform(head_pan, head_tilt):
    """
    Compute transformation matrix from head camera frame to robot base frame

    Stretch 3 head camera geometry (approximate):
    - Height above ground: ~1.1m when lift is at mid-position
    - Forward offset from base center: ~0.08m
    - Camera tilts down (negative angle) and pans left/right

    Args:
        head_pan: Head pan angle in radians (positive = counterclockwise)
        head_tilt: Head tilt angle in radians (positive = up)

    Returns:
        4x4 transformation matrix from camera frame to base frame
    """
    # Head camera mounting position relative to base (approximate)
    # These values should be calibrated for your specific robot
    camera_height_m = 1.10  # Height above ground
    camera_forward_offset_m = 0.08  # Forward from base center
    camera_lateral_offset_m = 0.0  # Left/right offset

    # Camera optical frame is:
    # X-right, Y-down, Z-forward (standard camera convention)
    # Robot base frame is:
    # X-forward, Y-left, Z-up

    # Rotation from camera optical frame to camera mounting frame
    # (This rotates from optical convention to base convention)
    R_mount_to_optical = Rotation.from_euler('xyz', [0, 0, -np.pi/2]).as_matrix()
    R_optical_to_mount = R_mount_to_optical.T

    # Head pan rotation (around Z-axis of base frame)
    R_pan = Rotation.from_euler('z', head_pan).as_matrix()

    # Head tilt rotation (around Y-axis after pan)
    R_tilt = Rotation.from_euler('y', -head_tilt).as_matrix()

    # Combined rotation: base -> pan -> tilt -> camera mounting -> camera optical
    R_base_to_camera = R_optical_to_mount @ R_tilt @ R_pan

    # Translation from base to camera
    # First apply pan to the forward/lateral offset, then add height
    pan_rotation_2d = np.array([[np.cos(head_pan), -np.sin(head_pan)],
                                 [np.sin(head_pan), np.cos(head_pan)]])

    offset_2d = pan_rotation_2d @ np.array([camera_forward_offset_m, camera_lateral_offset_m])

    t_base_to_camera = np.array([offset_2d[0], offset_2d[1], camera_height_m])

    # Build 4x4 transformation matrix
    T_base_to_camera = np.eye(4)
    T_base_to_camera[:3, :3] = R_base_to_camera
    T_base_to_camera[:3, 3] = t_base_to_camera

    # We need camera to base, so invert
    T_camera_to_base = np.linalg.inv(T_base_to_camera)

    return T_camera_to_base


def compute_end_effector_position(joint_state, gripper_length=0.18):
    """
    Compute end effector position in base frame using forward kinematics

    Simplified FK for Stretch 3:
    - Base is at origin
    - Lift raises vertically
    - Arm extends horizontally forward
    - Wrist yaw/pitch/roll adjust orientation
    - Gripper extends forward from wrist

    Args:
        joint_state: Dictionary with robot joint positions
        gripper_length: Length from wrist to gripper tip (meters)

    Returns:
        3D position of gripper tip in base frame [x, y, z]
    """
    # Extract relevant joint states
    lift_pos = joint_state['lift_pos']
    arm_pos = joint_state['arm_pos']
    wrist_yaw = joint_state['wrist_yaw_pos']
    wrist_pitch = joint_state['wrist_pitch_pos']

    # Stretch 3 geometry (approximate)
    base_to_lift_height = 0.1  # Height from base to bottom of lift
    lift_to_arm_offset = 0.0  # Lateral offset
    arm_base_forward_offset = 0.05  # Arm base is slightly forward of base center

    # Lift contributes to Z (vertical) position
    z = base_to_lift_height + lift_pos

    # Arm extension + gripper length projected by wrist angles
    # Wrist pitch: negative = pointing down
    # Wrist yaw: controls horizontal rotation

    # Total horizontal reach
    total_forward = arm_pos + gripper_length

    # Apply wrist pitch (vertical angle)
    horizontal_component = total_forward * np.cos(wrist_pitch)
    vertical_component = total_forward * np.sin(wrist_pitch)

    # Apply wrist yaw (horizontal rotation)
    x = arm_base_forward_offset + horizontal_component * np.cos(wrist_yaw)
    y = lift_to_arm_offset + horizontal_component * np.sin(wrist_yaw)
    z = z + vertical_component

    return np.array([x, y, z])


def transform_point_camera_to_base(point_camera, T_camera_to_base):
    """Transform a 3D point from camera frame to base frame"""
    point_homogeneous = np.append(point_camera, 1.0)
    point_base_homogeneous = T_camera_to_base @ point_homogeneous
    return point_base_homogeneous[:3]


####################################
# VISUALIZATION HELPERS
####################################

def draw_origin(image, camera_info, xyz, color, radius=6):
    """Draw a circle at 3D point projected to image"""
    # Project 3D to 2D
    camera_matrix = camera_info['camera_matrix']
    dist_coeffs = camera_info['distortion_coefficients']

    point_2d, _ = cv2.projectPoints(
        xyz.reshape(1, 3),
        np.zeros(3),  # No rotation (already in camera frame)
        np.zeros(3),  # No translation
        camera_matrix,
        dist_coeffs
    )

    center = tuple(point_2d[0, 0].astype(np.int32))
    cv2.circle(image, center, radius, color, -1, lineType=cv2.LINE_AA)
    return center


def draw_text(image, origin, text_lines):
    """Draw text at image location"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.5
    location = np.array(origin) + np.array([0, -55])
    location = location.astype(np.int32)

    for i, line in enumerate(text_lines):
        text_size = cv2.getTextSize(line, font, font_size, 4)
        (text_width, text_height), text_baseline = text_size
        center = int(text_width / 2)
        offset = np.array([-center, i * (1.7 * text_height)]).astype(np.int32)
        cv2.putText(image, line, tuple(location + offset), font, font_size,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, line, tuple(location + offset), font, font_size,
                    (255, 255, 255), 1, cv2.LINE_AA)


####################################
# ROBOT CONTROL PARAMETERS
####################################

# Initial pose for machine operation
joint_state_center = {
    'lift_pos': 0.85,  # Higher than default for better camera view
    'arm_pos': 0.25,  # Partially extended
    'wrist_yaw_pos': 0.0,
    'wrist_pitch_pos': -np.pi/4,  # Angled down for button/switch
    'wrist_roll_pos': 0.0,
    'gripper_pos': 10.0,  # Open
    'head_pan_pos': 0.0,  # Straight ahead
    'head_tilt_pos': -np.pi/6  # 30 degrees down
}

# Joint limits
min_joint_state = {
    'base_odom_theta': -1.5,
    'lift_pos': 0.2,
    'arm_pos': 0.0,
    'wrist_yaw_pos': -1.5,
    'wrist_pitch_pos': -1.4,
    'wrist_roll_pos': -1.5,
    'gripper_pos': 0.0
}

max_joint_state = {
    'base_odom_theta': 1.5,
    'lift_pos': 1.1,
    'arm_pos': 0.5,
    'wrist_yaw_pos': 1.5,
    'wrist_pitch_pos': 0.5,
    'wrist_roll_pos': 1.5,
    'gripper_pos': 50.0
}

# Velocity scaling
overall_velocity_scale = 0.8  # Slower for safety with base movement

joint_velocity_scale = {
    'base_forward': 0.3,  # NEW: Conservative base translation
    'base_counterclockwise': 2.0,
    'lift_up': 4.0,
    'arm_out': 3.0,
    'wrist_yaw_counterclockwise': 3.0,
    'wrist_pitch_up': 4.0,
    'wrist_roll_counterclockwise': 2.0,
    'gripper_open': 1.0
}

# Zero velocity
zero_vel = {
    'base_forward': 0.0,
    'base_counterclockwise': 0.0,
    'lift_up': 0.0,
    'arm_out': 0.0,
    'wrist_yaw_counterclockwise': 0.0,
    'wrist_pitch_up': 0.0,
    'wrist_roll_counterclockwise': 0.0,
    'gripper_open': 0.0
}

# Position to velocity mapping
pos_to_vel_cmd = {
    'base_odom_theta': 'base_counterclockwise',
    'lift_pos': 'lift_up',
    'arm_pos': 'arm_out',
    'wrist_yaw_pos': 'wrist_yaw_counterclockwise',
    'wrist_pitch_pos': 'wrist_pitch_up',
    'wrist_roll_pos': 'wrist_roll_counterclockwise',
    'gripper_pos': 'gripper_open'
}

vel_cmd_to_pos = {v: k for (k, v) in pos_to_vel_cmd.items()}

####################################
# BEHAVIOR PARAMETERS
####################################

# ArUco marker IDs
BUTTON_MARKER_ID = 100
SWITCH_MARKER_ID = 101

# Approach parameters
approach_threshold_m = 0.03  # Stop when within 3cm of target
position_error_tolerance = 0.05  # Acceptable error for "reached"

# Button press parameters
button_press_force_threshold = -15.0  # Gripper effort (Newtons)
button_press_depth_m = 0.02  # How far to push
button_press_duration_s = 1.0  # Hold duration

# Switch flip parameters
switch_grasp_threshold = 0.04  # Gripper width for successful grasp
switch_flip_angle = np.pi / 2  # 90 degree rotation
successful_grasp_effort = -10.0  # Gripper effort for grasp detection

# Safety parameters
max_approach_distance = 0.6  # Maximum distance to attempt approach
min_base_speed = 0.03  # Deadzone for base velocity
lost_target_timeout_frames = 15  # Stop if target not seen for this many frames

# Gripper length (from wrist to fingertips)
GRIPPER_LENGTH = 0.18  # meters


####################################
# MAIN DEMO LOGIC
####################################

def recenter_robot_for_machine_operation(robot):
    """Move robot to initial pose for machine operation"""
    print('Recentering robot for machine operation...')

    # Position head camera
    robot.head.move_to('head_pan', joint_state_center['head_pan_pos'])
    robot.head.move_to('head_tilt', joint_state_center['head_tilt_pos'])
    robot.push_command()
    robot.wait_command()
    time.sleep(0.5)

    # Position arm and wrist
    robot.lift.move_to(joint_state_center['lift_pos'])
    robot.push_command()
    robot.wait_command()
    time.sleep(0.5)

    robot.arm.move_to(joint_state_center['arm_pos'])
    robot.push_command()
    robot.wait_command()
    time.sleep(0.5)

    robot.end_of_arm.get_joint('wrist_yaw').move_to(joint_state_center['wrist_yaw_pos'])
    robot.end_of_arm.get_joint('wrist_pitch').move_to(joint_state_center['wrist_pitch_pos'])
    robot.end_of_arm.get_joint('wrist_roll').move_to(joint_state_center['wrist_roll_pos'])
    robot.push_command()
    robot.wait_command()
    time.sleep(0.5)

    # Open gripper
    robot.end_of_arm.get_joint('stretch_gripper').move_to(joint_state_center['gripper_pos'])
    robot.push_command()
    robot.wait_command()

    print('Robot centered and ready!')


def main(demo_mode='full'):
    """
    Main demo function

    Args:
        demo_mode: 'button_only', 'switch_only', or 'full'
    """

    # Initialize robot
    print('Initializing Stretch robot...')
    robot = rb.Robot()
    robot.startup()

    # Recenter robot
    recenter_robot_for_machine_operation(robot)

    # Initialize head camera
    print('Starting head camera...')
    pipeline, profile, camera_info, depth_scale = start_head_camera(robot, exposure='medium')

    # Initialize ArUco detector
    print('Initializing ArUco detector...')
    marker_info = {}
    with open('aruco_marker_info.yaml') as f:
        marker_info = yaml.load(f, Loader=SafeLoader)

    aruco_detector = ad.ArucoDetector(
        marker_info=marker_info,
        show_debug_images=True,
        use_apriltag_refinement=False,
        brighten_images=True
    )

    # Initialize velocity controller
    # Note: We'll use direct robot commands for base translation
    # since NormalizedVelocityControl may not support it yet
    controller = nvc.NormalizedVelocityControl(robot)
    controller.reset_base_odometry()

    # State machine
    if demo_mode == 'button_only':
        states = ['approach_button', 'press_button', 'retract', 'done']
    elif demo_mode == 'switch_only':
        states = ['approach_switch', 'flip_switch', 'retract', 'done']
    else:  # full
        states = ['approach_button', 'press_button', 'retract_button',
                  'approach_switch', 'flip_switch', 'retract_switch', 'done']

    current_state_idx = 0
    current_state = states[current_state_idx]

    frames_since_target_detected = 0
    button_press_start_time = None
    switch_grasp_achieved = False

    loop_timer = lt.LoopTimer()

    print(f'\nStarting demo in mode: {demo_mode}')
    print(f'Initial state: {current_state}')
    print('Press Ctrl+C to stop\n')

    try:
        while current_state != 'done':
            loop_timer.start_of_iteration()

            # Get camera frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            display_image = np.copy(color_image)

            # Detect ArUco markers
            aruco_detector.update(color_image, camera_info)
            markers = aruco_detector.get_detected_marker_dict()

            # Get current joint state
            joint_state = controller.get_joint_state()
            joint_state['base_odom_theta'] = hm.angle_diff_rad(
                joint_state['base_odom_theta'], 0.0
            )

            # Get head camera transform
            head_pan = joint_state.get('head_pan_pos', 0.0)
            head_tilt = joint_state.get('head_tilt_pos', -np.pi/6)
            T_camera_to_base = get_head_camera_to_base_transform(head_pan, head_tilt)

            # Compute end effector position
            ee_pos_base = compute_end_effector_position(joint_state, GRIPPER_LENGTH)

            # Find target based on current state
            target_camera = None
            target_name = None

            if 'button' in current_state:
                # Look for button marker
                for marker_id, marker in markers.items():
                    if marker_id == BUTTON_MARKER_ID or marker['info'].get('name') == 'button':
                        # Target is marker position (assuming marker is ON the button)
                        target_camera = marker['pos']
                        target_name = 'button'
                        frames_since_target_detected = 0
                        break

            elif 'switch' in current_state:
                # Look for switch marker
                for marker_id, marker in markers.items():
                    if marker_id == SWITCH_MARKER_ID or marker['info'].get('name') == 'switch':
                        target_camera = marker['pos']
                        target_name = 'switch'
                        frames_since_target_detected = 0
                        break

            if target_camera is None:
                frames_since_target_detected += 1

            # State machine execution
            cmd = zero_vel.copy()

            print(f'\n--- State: {current_state} ---')
            print(f'Target detected: {target_name is not None}')
            if target_camera is not None:
                print(f'Target position (camera): {target_camera * 100:.1f} cm')

            ###################
            # APPROACH STATES
            ###################
            if current_state in ['approach_button', 'approach_switch']:

                if target_camera is not None:
                    # Transform target to base frame
                    target_base = transform_point_camera_to_base(target_camera, T_camera_to_base)

                    # Compute error in base frame
                    position_error_base = target_base - ee_pos_base
                    error_magnitude = np.linalg.norm(position_error_base)

                    print(f'Target (base): [{target_base[0]:.3f}, {target_base[1]:.3f}, {target_base[2]:.3f}]')
                    print(f'EE (base): [{ee_pos_base[0]:.3f}, {ee_pos_base[1]:.3f}, {ee_pos_base[2]:.3f}]')
                    print(f'Error: {error_magnitude * 100:.1f} cm')

                    if error_magnitude < approach_threshold_m:
                        print('>>> TARGET REACHED! Transitioning to action state')
                        if current_state == 'approach_button':
                            current_state = 'press_button'
                            button_press_start_time = time.time()
                        else:
                            current_state = 'flip_switch'
                            switch_grasp_achieved = False

                    elif error_magnitude < max_approach_distance:
                        # Compute velocity commands
                        # Base frame: X=forward, Y=left, Z=up

                        # Forward/backward via base translation
                        base_forward_vel = position_error_base[0]

                        # Left/right via base rotation
                        base_rotation_vel = position_error_base[1] / max(abs(target_base[0]), 0.3)
                        if abs(base_rotation_vel) < min_base_speed:
                            base_rotation_vel = 0.0

                        # Up/down via lift
                        lift_vel = position_error_base[2]

                        # Arm stays mostly fixed (minor adjustments)
                        arm_vel = 0.0

                        # Wrist maintains downward angle
                        pitch_error = -np.pi/4 - joint_state['wrist_pitch_pos']
                        pitch_vel = pitch_error

                        yaw_error = 0.0 - joint_state['wrist_yaw_pos']
                        yaw_vel = yaw_error

                        cmd = {
                            'base_forward': base_forward_vel,
                            'base_counterclockwise': base_rotation_vel,
                            'lift_up': lift_vel,
                            'arm_out': arm_vel,
                            'wrist_pitch_up': pitch_vel,
                            'wrist_yaw_counterclockwise': yaw_vel,
                            'gripper_open': 0.5  # Keep gripper open
                        }

                    # Visualize
                    draw_origin(display_image, camera_info, target_camera, (0, 255, 0), 8)
                    draw_text(display_image, (320, 240), [
                        f'{target_name.upper()}',
                        f'Error: {error_magnitude * 100:.1f} cm',
                        f'State: {current_state}'
                    ])

                else:
                    if frames_since_target_detected > lost_target_timeout_frames:
                        print('>>> TARGET LOST! Stopping.')
                        cmd = zero_vel.copy()

            ###################
            # PRESS BUTTON
            ###################
            elif current_state == 'press_button':
                # Apply downward force
                press_duration = time.time() - button_press_start_time

                if press_duration < button_press_duration_s:
                    # Push down with lift
                    cmd = {
                        'lift_up': -0.3,  # Slow downward motion
                        'gripper_open': -0.5  # Close gripper for contact
                    }
                    print(f'Pressing button... ({press_duration:.1f}s / {button_press_duration_s:.1f}s)')
                else:
                    print('>>> BUTTON PRESSED! Transitioning to retract')
                    current_state_idx += 1
                    if current_state_idx < len(states):
                        current_state = states[current_state_idx]
                    cmd = zero_vel.copy()

            ###################
            # FLIP SWITCH
            ###################
            elif current_state == 'flip_switch':
                gripper_effort = joint_state.get('gripper_eff', 0.0)

                if not switch_grasp_achieved:
                    # Close gripper to grasp switch
                    cmd = {
                        'gripper_open': -1.0  # Close
                    }

                    if gripper_effort < successful_grasp_effort:
                        print('>>> SWITCH GRASPED!')
                        switch_grasp_achieved = True
                    else:
                        print(f'Grasping switch... (effort: {gripper_effort:.1f} N)')
                else:
                    # Flip switch with wrist roll
                    current_roll = joint_state['wrist_roll_pos']
                    target_roll = switch_flip_angle

                    if abs(current_roll - target_roll) > 0.1:
                        cmd = {
                            'wrist_roll_counterclockwise': 0.5
                        }
                        print(f'Flipping switch... ({current_roll * 180 / np.pi:.0f}° / {target_roll * 180 / np.pi:.0f}°)')
                    else:
                        print('>>> SWITCH FLIPPED! Transitioning to retract')
                        current_state_idx += 1
                        if current_state_idx < len(states):
                            current_state = states[current_state_idx]
                        cmd = zero_vel.copy()

            ###################
            # RETRACT
            ###################
            elif current_state.startswith('retract'):
                # Move back and up
                if joint_state['arm_pos'] > min_joint_state['arm_pos'] + 0.05:
                    cmd = {
                        'arm_out': -0.5,  # Retract arm
                        'lift_up': 0.2,  # Slight upward
                        'gripper_open': 1.0  # Open gripper
                    }
                    print('Retracting...')
                else:
                    print('>>> RETRACT COMPLETE!')
                    current_state_idx += 1
                    if current_state_idx < len(states):
                        current_state = states[current_state_idx]
                    else:
                        current_state = 'done'
                    cmd = zero_vel.copy()

            # Apply velocity scaling and joint limits
            if cmd != zero_vel:
                cmd = {k: overall_velocity_scale * v for k, v in cmd.items()}
                cmd = {k: joint_velocity_scale.get(k, 1.0) * v for k, v in cmd.items()}

                # Enforce joint limits (for non-base commands)
                for cmd_name, vel in list(cmd.items()):
                    if cmd_name in vel_cmd_to_pos:
                        pos_name = vel_cmd_to_pos[cmd_name]
                        if pos_name in joint_state:
                            pos = joint_state[pos_name]
                            if vel < 0.0 and pos < min_joint_state.get(pos_name, -np.inf):
                                cmd[cmd_name] = 0.0
                            elif vel > 0.0 and pos > max_joint_state.get(pos_name, np.inf):
                                cmd[cmd_name] = 0.0

            # Send command
            # Note: base_forward is not in standard NormalizedVelocityControl
            # We'll use direct robot.base.set_velocity for base translation
            if 'base_forward' in cmd:
                base_forward = cmd.pop('base_forward')
                base_rotation = cmd.get('base_counterclockwise', 0.0)

                # Convert normalized velocity to m/s
                max_base_vel = 0.3  # m/s
                v_m = base_forward * max_base_vel
                w_r = base_rotation * 0.5  # rad/s

                robot.base.set_velocity(v_m, w_r)

            if cmd:
                controller.set_command(cmd)

            # Display
            cv2.putText(display_image, f'State: {current_state}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Head Camera - Machine Operation', display_image)
            cv2.waitKey(1)

            loop_timer.end_of_iteration()
            loop_timer.pretty_print(minimum=True)

    except KeyboardInterrupt:
        print('\n\nInterrupted by user!')

    finally:
        print('\nStopping robot...')
        controller.stop()
        robot.stop()
        pipeline.stop()
        cv2.destroyAllWindows()
        print('Demo complete!')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='Head Camera Machine Operation Demo',
        description='Autonomous button pressing and switch flipping using head camera and ArUco markers'
    )

    parser.add_argument(
        '-m', '--mode',
        type=str,
        default='full',
        choices=['button_only', 'switch_only', 'full'],
        help='Demo mode: button_only, switch_only, or full (default)'
    )

    args = parser.parse_args()

    main(demo_mode=args.mode)
