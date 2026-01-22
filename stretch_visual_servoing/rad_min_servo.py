import d405_helpers as dh
import pyrealsense2 as rs
import numpy as np
import cv2
import normalized_velocity_control as nvc
import stretch_body.robot as rb
import time
import aruco_detector as ad
import yaml
from yaml.loader import SafeLoader
from scipy.spatial.transform import Rotation
from hello_helpers import hello_misc as hm
import argparse
import zmq
import loop_timer as lt
import yolo_networking as yn
from stretch_body import robot_params
from stretch_body import hello_utils as hu

def draw_origin(image, camera_info, origin_xyz, color):
    radius = 6
    thickness = -1
    center = np.round(dh.pixel_from_3d(origin_xyz, camera_info)).astype(np.int32)
    cv2.circle(image, center, radius, color, -1, lineType=cv2.LINE_AA)

    
def draw_text(image, origin, text_lines):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.5
    location = origin + np.array([0, -55])
    location = location.astype(np.int32)
        
    for i, line in enumerate(text_lines):
        text_size = cv2.getTextSize(line, font, font_size, 4)
        (text_width, text_height), text_baseline = text_size
        center = int(text_width / 2)
        offset = np.array([-center, i * (1.7*text_height)]).astype(np.int32)
        cv2.putText(image, line, location + offset, font, font_size, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, line, location + offset, font, font_size, (255, 255, 255), 1, cv2.LINE_AA)

def get_dxl_joint_limits(joint):
    # method to get dynamixel joint limits in radians from robot params
    # Refer https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_hello_XL430.py#L1196:L1199

    range_t = robot_params.RobotParams().get_params()[1][joint]['range_t']
    flip_encoder_polarity = robot_params.RobotParams().get_params()[1][joint]['flip_encoder_polarity']
    gr = robot_params.RobotParams().get_params()[1][joint]['gr']
    zero_t = robot_params.RobotParams().get_params()[1][joint]['zero_t']

    polarity = -1.0 if flip_encoder_polarity else 1.0
    range_rad = []
    for t in range_t:
        x = t - zero_t
        rad_world = polarity*hu.deg_to_rad((360.0 * x / 4096.0))/gr
        range_rad.append(rad_world)
    return range_rad
    
####################################
# Miscellaneous Parameters

motion_on = True
print_timing = True

stop_if_toy_not_detected_this_many_frames = 10 #4 #1

# Lock behavior parameters
lock_wrist_rotation_rad = np.pi / 4  # 45 degrees counterclockwise
lock_error_threshold = 0.25  # 25 cm - if we were this close before losing detection, proceed to lock

max_retract_state_count = 60

# Defines a deadzone for mobile base rotation, since low values can
# lead to no motion and noises on some surfaces like carpets.
min_base_speed = 0.05

####################################
## Control Loop Frequency Regulation

# Target control loop rate used when receiving task-relevant
# information from an external process, instead of directly acquiring
# images from the D405. Directly acquiring images uses a blocking call
# and the D405 provides images at a consistent rate, which regulates
# the control loop. Receiving task-relevant information from an
# external process uses nonblocking polling of communications that can
# be highly variable due to communication and computation timing. The
# polling timeout is set automatically in an attempt to approximate
# this target control loop frequency.
target_control_loop_rate_hz = 15
# Proportional gain used to regulate the polling timeout to achieve the target frequency
timeout_proportional_gain = 0.1
# How much history to consider when regulating the polling timeout.
seconds_of_timing_history = 1

####################################

## Grasp Parameters

# Toy Cube with ArUco Marker
toy_depth_m = 0.055
toy_width_m = 0.0542

stop_if_error_below_this = 0.16

# Find a way to make the gripper faster? These are the maximum
# available velocities.
gripper_open_speed = 1.0
gripper_close_speed = 1.0

lost_ball_target_error_too_large = 0.10
lost_ball_fingertips_too_close = 0.038

successful_grasp_effort = -14.0
successful_grasp_max_fingertip_distance = 0.085 #0.075 m
successful_grasp_min_fingertip_distance = 0.05 #0.03 m

# Approximation based on just_touching key statefrom gripper characterization
default_between_fingertips = np.array([0.01, 0.035, 0.17])
distance_between_fully_open_fingertips = 0.16
max_toy_z_for_default_fingertips = 0.12

# TODO: update tool offset values
# x: left/right, y: up/down, z: in/out
tool_offset = np.array([0.0, 0.1, 0.35])

####################################
## Gains for Reach Behavior

max_distance_for_attempted_reach = 1.0

arm_retraction_speedup = 5.0

max_gripper_length = 0.26

overall_visual_servoing_velocity_scale = 1.0

joint_visual_servoing_velocity_scale = {
    'base_counterclockwise' : 4.0,
    'lift_up' : 6.0,
    'arm_out' : 6.0,
    'wrist_yaw_counterclockwise' : 4.0,
    'wrist_pitch_up' : 6.0,
    'wrist_roll_counterclockwise': 1.0,
    'gripper_open' : 1.0
}

####################################
## Initial Pose

joint_state_center = {
    'lift_pos' : 0.7,
    'arm_pos': 0.01,
    'wrist_yaw_pos': 0.0,
    'wrist_pitch_pos': 0.0, #-0.6
    'wrist_roll_pos': 0.0,
    'gripper_pos': 0
}

####################################
## Allowed Range of Motion

min_joint_state = {
    'base_odom_theta' : -0.8,
    'lift_pos': 0.1,
    'arm_pos': 0.01, #0.03
    'wrist_yaw_pos': -0.20, #-0.25
    'wrist_pitch_pos': -1.2,
    'wrist_roll_pos': -0.1,
    'gripper_pos' : 3.0 #3.5 #4.0 #3.0 
    }

max_joint_state = {
    'base_odom_theta' : 0.8,
    'lift_pos': 1.05, #
    'arm_pos': 0.45,
    'wrist_yaw_pos': 1.0, #0.5
    'wrist_pitch_pos': 0.2, #-0.4
    'wrist_roll_pos': 0.1,
    'gripper_pos': get_dxl_joint_limits('stretch_gripper')[1] #10.46
    }


####################################
## Zero Velocity Command

zero_vel = {
    'base_counterclockwise': 0.0,
    'lift_up': 0.0,
    'arm_out': 0.0,
    'wrist_yaw_counterclockwise': 0.0,
    'wrist_pitch_up': 0.0,
    'wrist_roll_counterclockwise': 0.0,
    'gripper_open': 0.0
}

####################################
## Translate Between Keys

pos_to_vel_cmd = {
    'base_odom_theta' : 'base_counterclockwise', 
    'lift_pos':'lift_up', 
    'arm_pos':'arm_out',
    'wrist_yaw_pos':'wrist_yaw_counterclockwise',
    'wrist_pitch_pos':'wrist_pitch_up',
    'wrist_roll_pos':'wrist_roll_counterclockwise',
    'gripper_pos':'gripper_open'
}

vel_cmd_to_pos = { v:k for (k,v) in pos_to_vel_cmd.items() }

####################################

class RegulatePollTimeout:
    def __init__(self, target_control_loop_rate_hz, seconds_of_timing_history, timeout_proportional_gain, debug_on=False):

        self.debug_on = debug_on
        self.target_control_loop_rate_hz = target_control_loop_rate_hz
        self.seconds_of_timing_history = seconds_of_timing_history
        self.timeout_proportional_gain = timeout_proportional_gain

        self.target_control_loop_period_ms = 1000.0 * (1.0/self.target_control_loop_rate_hz)
        self.initial_timeout_for_socket_poll_ms = self.target_control_loop_period_ms
        self.timeout_for_socket_poll_ms = self.target_control_loop_period_ms
        
        self.recent_polling_durations_max_length = self.seconds_of_timing_history * int(round(self.target_control_loop_rate_hz))
        self.recent_non_polling_durations_max_length = self.seconds_of_timing_history * int(round(self.target_control_loop_rate_hz))
        
        self.time_before_socket_poll = None
        self.prev_time_before_socket_poll = None
        self.time_after_socket_poll = None
        self.prev_time_after_socket_poll = None
        
        self.recent_polling_durations = []
        self.recent_non_polling_durations = []
        
    def run_after_polling(self):
        self.prev_time_after_socket_poll = self.time_after_socket_poll
        self.time_after_socket_poll = time.time()

    def get_poll_timeout(self): 
        # When obtaining task-relevant information via a
        # socket, the required processing should be low. Only
        # robot communication is likely to take significant
        # time. Consequently, the timeout for polling is
        # expected to represent a majority of the period for
        # the control loop. This attempts to select a polling
        # timeout that will result in the control loop being
        # close to the target frequency. Ultimately,
        # performance will depend on the rate at which
        # task-relevant information is received, but motor
        # control behavior will be more consistent.
        
        self.prev_time_before_socket_poll = self.time_before_socket_poll
        self.time_before_socket_poll = time.time()
        
        mean_polling_duration_ms = None
        mean_non_polling_duration_ms = None

        if self.debug_on: 
            print('--------------------------------------------------')
            print('RegulatePollTimeout: get_poll_timeout()')
            print('self.initial_timeout_for_socket_poll_ms =', self.initial_timeout_for_socket_poll_ms)
        
        if (self.time_after_socket_poll is not None) and (self.prev_time_before_socket_poll is not None):
            self.recent_polling_durations.append(self.time_after_socket_poll - self.prev_time_before_socket_poll)
            if len(self.recent_polling_durations) > self.recent_polling_durations_max_length:
                self.recent_polling_durations.pop(0)
            mean_polling_duration_ms = 1000.0 * np.mean(np.array(self.recent_polling_durations))
            if self.debug_on: 
                print('mean_polling_duration_ms =', mean_polling_duration_ms)

        if (self.time_after_socket_poll is not None) and (self.time_before_socket_poll is not None):
            self.recent_non_polling_durations.append(self.time_before_socket_poll - self.time_after_socket_poll)                   
            if len(self.recent_non_polling_durations) > self.recent_non_polling_durations_max_length:
                self.recent_non_polling_durations.pop(0)
            mean_non_polling_duration_ms = 1000.0 * np.mean(np.array(self.recent_non_polling_durations))
            if self.debug_on: 
                print('mean_non_polling_duration_ms =', mean_non_polling_duration_ms)

        if (mean_polling_duration_ms is not None) and (mean_non_polling_duration_ms is not None):
            mean_full_duration_ms = mean_polling_duration_ms + mean_non_polling_duration_ms
            full_duration_error_ms = self.target_control_loop_period_ms - mean_full_duration_ms
            self.timeout_for_socket_poll_ms = self.timeout_for_socket_poll_ms + (self.timeout_proportional_gain * full_duration_error_ms)

            if self.debug_on: 
                print('self.target_control_loop_perios_ms =', self.target_control_loop_period_ms)
                print('mean_full_duration_ms =', mean_full_duration_ms)
                print('full_duration_error_ms =', full_duration_error_ms)
                print('self.timeout_proportional_gain =', self.timeout_proportional_gain)
                print('self.timeout_for_socket_poll_ms =', self.timeout_for_socket_poll_ms)

        timeout_for_socket_poll_ms_int = int(round(self.timeout_for_socket_poll_ms))
        if timeout_for_socket_poll_ms_int <= 0:
            timeout_for_socket_poll_ms_int = 1

        if self.debug_on: 
            print('timeout_for_socket_poll_ms_int =', timeout_for_socket_poll_ms_int)
            print('--------------------------------------------------')
        return timeout_for_socket_poll_ms_int


def recenter_robot(robot):
    pan = np.pi/2.0
    tilt = -np.pi/2.0
    robot.head.move_to('head_pan', pan)
    robot.head.move_to('head_tilt', tilt)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.get_joint('wrist_yaw').move_to(joint_state_center['wrist_yaw_pos'])
    robot.end_of_arm.get_joint('wrist_pitch').move_to(joint_state_center['wrist_pitch_pos'])
    robot.push_command()
    robot.wait_command()

    robot.arm.move_to(joint_state_center['arm_pos'])
    robot.push_command()
    robot.wait_command()
    
    robot.end_of_arm.get_joint('wrist_roll').move_to(joint_state_center['wrist_roll_pos'])
    robot.push_command()
    robot.wait_command()
    
    robot.lift.move_to(1.05)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.get_joint('stretch_gripper').move_to(joint_state_center['gripper_pos'])
    robot.push_command()
    robot.wait_command()
        

def main(use_yolo, use_remote_computer, exposure):
    try:
        
        robot = rb.Robot()
        robot.startup()
        recenter_robot(robot)
        controller = nvc.NormalizedVelocityControl(robot)
        controller.reset_base_odometry()

        marker_info = {}
        with open('aruco_marker_info.yaml') as f:
            marker_info = yaml.load(f, Loader=SafeLoader)

        detect_ball_on = False
        detect_aruco_toy_on = True
        aruco_detector = ad.ArucoDetector(marker_info=marker_info, show_debug_images=True, use_apriltag_refinement=False, brighten_images=True)

        first_frame = True

        behavior = 'reach'
        prev_behavior = 'reach'
        grasping_the_target = False
        pre_reach = True
        last_target_error = None  # Track the last known target error before detection was lost

        # TODO: add code that closes the gripper as much as possible

        pipeline, profile = dh.start_d405(exposure)

        frames_since_toy_detected = 0
            
        loop_timer = lt.LoopTimer()
        
        while True:
            loop_timer.start_of_iteration()

            toy_target = None
            
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if (not depth_frame) or (not color_frame):
                continue

            if first_frame:
                depth_scale = dh.get_depth_scale(profile)
                print('depth_scale =', depth_scale)
                print()

                depth_camera_info = dh.get_camera_info(depth_frame)
                color_camera_info = dh.get_camera_info(color_frame)
                camera_info = depth_camera_info
                #camera_info = color_camera_info
                print_camera_info = True
                if print_camera_info: 
                    for camera_info, name in [(depth_camera_info, 'depth'), (color_camera_info, 'color')]:
                        print(name + ' camera_info:')
                        print(camera_info)
                        print()

                first_frame = False
                
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            image = np.copy(color_image)

            if detect_aruco_toy_on:                                                         
                aruco_detector.update(color_image, camera_info)                             
                markers = aruco_detector.get_detected_marker_dict()                         
                                                                                            
                button_left_pos = None                                                      
                button_right_pos = None                                                     
                for k in markers:                                                           
                    m = markers[k]                                                          
                    name = m['info']['name']                                                
                    if name == 'button_left':                                               
                        button_left_pos = m['pos']                                          
                    elif name == 'button_right':                                            
                        button_right_pos = m['pos']                                         
                                                                                            
                # Calculate midpoint if both markers detected                               
                if button_left_pos is not None and button_right_pos is not None:            
                    toy_target = (button_left_pos + button_right_pos) / 2.0   

            print()

            toy_name = 'ArUco Cube'
            if toy_target is None:
                print(toy_name + ' Detection: FAILED')
            else:
                print(toy_name + ' Detection: SUCCEEDED')

            joint_state = controller.get_joint_state()
            # convert base odometry angle to be in the range -pi to pi
            joint_state['base_odom_theta'] = hm.angle_diff_rad(joint_state['base_odom_theta'], 0.0)

            print('gripper effort = {:.2f}'.format(joint_state['gripper_eff']))

            position_error = toy_target - tool_offset
            target_error = np.linalg.norm(position_error)

            print('behavior =', behavior)
            print('pre_reach =', pre_reach)
                        
            if behavior == 'reach':

                prev_behavior = behavior

                if pre_reach:
                    cmd = {}

                    gripper_ready = False
                    if joint_state['gripper_pos'] >= (0.9 * max_joint_state['gripper_pos']):
                        gripper_ready = True
                        cmd['gripper_open'] = 0.0
                    elif not grasping_the_target:
                        cmd['gripper_open'] = gripper_open_speed

                    cmd['wrist_pitch_up'] = 0.0

                    if gripper_ready:
                        pre_reach = False
                        cmd = zero_vel.copy()

                    if cmd:
                        cmd = {k: overall_visual_servoing_velocity_scale * v for (k,v) in cmd.items()}
                        cmd = {k: joint_visual_servoing_velocity_scale[k] * v for (k,v) in cmd.items()}

                        cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        controller.set_command(cmd)

                elif (toy_target is not None) and (target_error <= max_distance_for_attempted_reach):            

                    x_error, y_error, z_error = position_error

                    yaw_velocity = -x_error
                    pitch_velocity = -y_error

                    roll_velocity =  0.0 - joint_state['wrist_roll_pos']

                    # Transform camera frame errors into errors for the Cartesian joints
                    yaw = joint_state['wrist_yaw_pos']
                    pitch = -joint_state['wrist_pitch_pos']
                    roll = -joint_state['wrist_roll_pos']
                    r = Rotation.from_euler('yxz', [yaw, pitch, roll]).as_matrix()
                    rotated_lift = np.matmul(r, np.array([0.0, -1.0, 0.0]))
                    rotated_arm = np.matmul(r, np.array([0.0, 0.0, 1.0]))
                    rotated_base = np.matmul(r, np.array([-1.0, 0.0, 0.0]))

                    lift_velocity = np.dot(rotated_lift, position_error)
                    arm_velocity = np.dot(rotated_arm, position_error)

                    #base_rotational_velocity = np.dot(rotated_base, position_error) / (joint_state['arm_pos'] + max_gripper_length)
                    base_rotational_velocity = np.dot(rotated_base, position_error)
                    #print('base_rotational_velocity =', base_rotational_velocity)
                    if abs(base_rotational_velocity) < min_base_speed:
                        base_rotational_velocity = 0.0

                    #print('base_rotational_velocity =', base_rotational_velocity)
                    #print('base_odom_theta =', joint_state['base_odom_theta'])

                    if arm_velocity < 0.0:
                        arm_velocity = arm_retraction_speedup * arm_velocity

                    cmd = {
                        'lift_up' : lift_velocity,
                        'arm_out' : arm_velocity,
                        'wrist_yaw_counterclockwise' : yaw_velocity,
                        'wrist_pitch_up' : pitch_velocity,
                        'wrist_roll_counterclockwise' : roll_velocity,
                        'base_counterclockwise' : base_rotational_velocity
                    }


                    # if target_error < grasp_if_error_below_this:
                    #     cmd['gripper_open'] = 0.0

                    #     if ((not grasping_the_target) and
                    #         (joint_state['gripper_eff'] < successful_grasp_effort) and
                    #         (distance_between_fingertips < successful_grasp_max_fingertip_distance) and
                    #         (distance_between_fingertips > successful_grasp_min_fingertip_distance)):
                    #         print('I GOT THE BALL!!!')
                    #         grasping_the_target = True
                    #         behavior = 'retract'
                    # else:
                    #     cmd['gripper_open'] = gripper_open_speed

                    if target_error < stop_if_error_below_this:
                        cmd = zero_vel.copy()

                    cmd = {k: overall_visual_servoing_velocity_scale * v for (k,v) in cmd.items()}
                    cmd = {k: joint_visual_servoing_velocity_scale[k] * v for (k,v) in cmd.items()}

                    if motion_on:
                        cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        controller.set_command(cmd)

                else:
                    # if not reaching and toy_target...?
                    joint_state = controller.get_joint_state()
                    stop_joints = zero_vel.copy()

                    # Check if we should transition to lock behavior
                    # If we lost detection but were close enough, proceed to lock the knob
                    if (last_target_error is not None and
                        last_target_error < lock_error_threshold and
                        frames_since_toy_detected >= stop_if_toy_not_detected_this_many_frames):
                        print('Target lost but was close enough - transitioning to LOCK behavior')
                        behavior = 'lock'
                        cmd = zero_vel.copy()
                    elif frames_since_toy_detected >= stop_if_toy_not_detected_this_many_frames:
                        cmd = stop_joints
                        cmd['gripper_open'] = gripper_open_speed
                    elif frames_since_fingers_detected >= stop_if_fingers_not_detected_this_many_frames:
                        cmd = stop_joints
                    else:
                        # Stop at Boundaries
                        cmd = { k:v for (k,v) in stop_joints.items() if (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]]) }
                        cmd = { k:v for (k,v) in stop_joints.items() if (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]]) }

                    if cmd:
                        cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        controller.set_command(cmd)

            elif behavior == 'lock':
                # Lock behavior: twist the knob
                # Phase 1: rotate left 45 degrees
                # Phase 2: wait 1 second
                # Phase 3: rotate right 90 degrees
                if prev_behavior != 'lock':
                    lock_state_count = 0
                    lock_phase = 'left'
                    lock_initial_roll = joint_state['wrist_roll_pos']
                    lock_target_roll = lock_initial_roll - lock_wrist_rotation_rad  # Left 45 degrees
                    lock_wait_start = None
                    print(f'LOCK: Phase 1 - Rotating left to {np.degrees(lock_target_roll):.1f} degrees')
                prev_behavior = behavior

                with controller.lock:
                    current_roll = joint_state['wrist_roll_pos']

                    if lock_phase == 'left':
                        roll_error = abs(lock_target_roll - current_roll)
                        if roll_error < 0.05:  # Within ~3 degrees tolerance
                            lock_phase = 'wait'
                            lock_wait_start = time.time()
                            print('LOCK: Phase 2 - Waiting 1 second')
                        else:
                            robot.end_of_arm.get_joint('wrist_roll').move_to(lock_target_roll, v_des=1.0, a_des=2.0)

                    elif lock_phase == 'wait':
                        if time.time() - lock_wait_start >= 1.0:
                            lock_phase = 'right'
                            # Right 90 degrees from the left position (which is 45 degrees left of initial)
                            # So target is initial + 45 degrees to the right
                            lock_target_roll = lock_initial_roll + lock_wrist_rotation_rad
                            print(f'LOCK: Phase 3 - Rotating right to {np.degrees(lock_target_roll):.1f} degrees')

                    elif lock_phase == 'right':
                        roll_error = abs(lock_target_roll - current_roll)
                        if roll_error < 0.05:  # Within ~3 degrees tolerance
                            lock_phase = 'complete'
                            print('LOCK: Twist complete!')
                        else:
                            robot.end_of_arm.get_joint('wrist_roll').move_to(lock_target_roll, v_des=1.0, a_des=2.0)

                    robot.push_command()

                # After lock is complete or timeout, transition back to reach
                if lock_phase == 'complete' or lock_state_count > 90:  # ~6 seconds timeout at 15Hz
                    cmd = zero_vel.copy()
                    behavior = 'reach'
                    pre_reach = True
                    last_target_error = None  # Reset for next reach attempt
                    print('LOCK: Behavior complete, returning to reach')

                lock_state_count = lock_state_count + 1

            if not use_yolo:

                if toy_target is not None:
                    # draw blue circle for the toy target position
                    draw_origin(image, camera_info, toy_target, (255, 0, 0))
                    x,y,z = toy_target * 100.0
                    width = toy_width_m
                    text_lines = [
                        "{:.1f} cm wide".format(width*100.0),
                        "{:.1f}, {:.1f}, {:.1f} cm".format(x,y,z)
                        ]
                    
                    center = np.round(dh.pixel_from_3d(toy_target, camera_info)).astype(np.int32)
                    draw_text(image, center, text_lines)

                if between_fingertips is not None: 
                    # draw white circle for point between fingertip
                    draw_origin(image, camera_info, between_fingertips, (255, 255, 255))

                
                aruco_to_fingertips.draw_fingertip_frames(fingertips,
                                                          image,
                                                          camera_info,
                                                          axis_length_in_m=0.02,
                                                          draw_origins=True,
                                                          write_coordinates=True)
                

                
                cv2.imshow('Features Used for Visual Servoing', image)
            cv2.waitKey(1)

            loop_timer.end_of_iteration()
            if print_timing: 
                loop_timer.pretty_print(minimum=True)
    finally:
        controller.stop()
        robot.stop()
        pipeline.stop()




if __name__ == '__main__':

    
    parser = argparse.ArgumentParser(
        prog='Stretch 3 Visual Servoing Demo',
        description='This application provides a demonstration of interactive grasping using visual servoing and the gripper-mounted D405.',
    )
    parser.add_argument('-y', '--yolo', action='store_true', help = 'Receive task-relevant features for visual servoing from an external process using YOLOv8. The default is to servo to a cube with an ArUco marker with a single process using OpenCV. To use YOLOv8, you will need to use this option. You will also need to run send_d405_images.py and recv_and_yolo_d405_images.py. Prior to using this option, configure the network with the file yolo_networking.py.')

    parser.add_argument('-r', '--remote', action='store_true', help = 'Use this argument when allowing a remote computer to send task-relevant information for visual servoing, such as 3D positions for the fingertips and target objects. Prior to using this option, configure the network with the file yolo_networking.py.')

    parser.add_argument('-e', '--exposure', action='store', type=str, default='low', help=f'Set the D405 exposure to {dh.exposure_keywords} or an integer in the range {dh.exposure_range}') 
            
    
    args = parser.parse_args()
    use_yolo = args.yolo
    use_remote_computer = args.remote

    exposure = args.exposure

    if not dh.exposure_argument_is_valid(exposure):
        raise argparse.ArgumentTypeError(f'The provided exposure setting, {exposure}, is not a valide keyword, {dh.exposure_keywords}, or is outside of the allowed numeric range, {dh.exposure_range}.')    
    
    main(use_yolo, use_remote_computer, exposure)
