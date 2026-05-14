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

stop_if_target_not_detected_this_many_frames = 10 #4 #1
stop_if_fingers_not_detected_this_many_frames = 10 #4 #1

# Lock behavior parameters
lock_error_threshold = 0.15  # 25 cm - if we were this close before losing detection, proceed to lock

# Defines a deadzone for mobile base rotation, since low values can
# lead to no motion and noises on some surfaces like carpets.
min_base_speed = 0.05

####################################

## Button Pressing Parameters

# Target width for display purposes only
target_width_m = 0.0542

# Distance threshold to trigger button press (lock behavior)
grasp_if_error_below_this = 0.1

# Gripper speed when opening at start
gripper_open_speed = 1.0

# Fallback fingertip position when markers are occluded
default_between_fingertips = np.array([0.01, 0.035, 0.17])
distance_between_fully_open_fingertips = 0.16
max_target_z_for_default_fingertips = 0.12

####################################
## Gains for Reach Behavior

max_distance_for_attempted_reach = 1.0

arm_retraction_speedup = 5.0

max_gripper_length = 0.26

overall_visual_servoing_velocity_scale = 0.5

joint_visual_servoing_velocity_scale = {
    'base_counterclockwise' : 4.0,
    'lift_up' : 6.0,
    'arm_out' : 6.0,
    'wrist_yaw_counterclockwise' : 4.0,
    'wrist_pitch_up' : 6.0,
    'wrist_roll_counterclockwise': 0.25,  # Slowed down for dial twisting
    'gripper_open' : 1.0
}

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

####################################
## Allowed Range of Motion

min_joint_state = {
    'base_odom_theta' : -0.8,
    'lift_pos': 0.1,
    'arm_pos': 0.01, #0.03
    'wrist_yaw_pos': -0.20, #-0.25
    'wrist_pitch_pos': -1.2,
    'wrist_roll_pos': -1.6,  # Allow larger range for dial twisting
    'gripper_pos' : 3.0 #3.5 #4.0 #3.0 
    }

max_joint_state = {
    'base_odom_theta' : 0.8,
    'lift_pos': 1.05, #
    'arm_pos': 0.45,
    'wrist_yaw_pos': 1.0, #0.5
    'wrist_pitch_pos': 0.2, #-0.4
    'wrist_roll_pos': 1.6,  # Allow larger range for dial twisting
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
    
    robot.lift.move_to(1.02)
    robot.push_command()
    robot.wait_command()

    # robot.end_of_arm.get_joint('stretch_gripper').move_to(joint_state_center['gripper_pos'])
    # robot.push_command()
    # robot.wait_command()
        

def main(exposure, mop):
    try:
        
        robot = rb.Robot()
        robot.startup()
        recenter_robot(robot)
        controller = nvc.NormalizedVelocityControl(robot)
        controller.reset_base_odometry()

        marker_info = {}
        with open('aruco_marker_info.yaml') as f:
            marker_info = yaml.load(f, Loader=SafeLoader)

        detect_aruco_button_on = True
        aruco_detector = ad.ArucoDetector(marker_info=marker_info, show_debug_images=True, use_apriltag_refinement=False, brighten_images=True)
        aruco_to_fingertips = af.ArucoToFingertips(default_height_above_mounting_surface=af.suctioncup_height['cup_bottom'])

        first_frame = True

        behavior = 'reach'
        prev_behavior = 'reach'
        pre_reach = True
        last_target_error = None  # Track the last known target error before detection was lost

        # Assume that the gripper starts out fully opened
        distance_between_fingertips = distance_between_fully_open_fingertips
        prev_distance_between_fingertips = distance_between_fully_open_fingertips

        pipeline, profile = dh.start_d405(exposure)

        frames_since_target_detected = 0
        frames_since_fingers_detected = 0
            
        loop_timer = lt.LoopTimer()

        fingertips = {}
        
        while True:
            loop_timer.start_of_iteration()

            toy_target = None
            fingertip_left_pos = None       
            fingertip_right_pos = None
            between_fingertips = None
            distance_between_fingertips = None
            
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

            if detect_aruco_button_on:                                                         
                aruco_detector.update(color_image, camera_info)                             
                markers = aruco_detector.get_detected_marker_dict()                         
                fingertips = aruco_to_fingertips.get_fingertips(markers)                    
                                                                                            
                tag_6_pos = None                                                      
                tag_5_pos = None                                                     
                for k in markers:                                                           
                    m = markers[k]                                                          
                    if k == 6:
                        tag_6_pos = m['pos']                                          
                    elif k == 5:
                        tag_5_pos = m['pos']                                         
                                                                                            
                # Calculate midpoint if both markers detected (tag 6 on left, tag 5 on right)                               
                if tag_6_pos is not None and tag_5_pos is not None:            
                    toy_target = (tag_6_pos + tag_5_pos) / 2.0   

            print()

            target_name = 'Dial Target (Tags 6 & 5)'
            if toy_target is None:
                print(target_name + ' Detection: FAILED')
            else:
                print(target_name + ' Detection: SUCCEEDED')
 
            # fingertip_left_pose = None
            # fingertip_right_pose = None
            # f = fingertips.get('left', None)
            # if f is not None:
            #     fingertip_left_pos = f['pos']
            #     print('Left Finger ArUco Marker Detection: SUCCEEDED')
            # else:
            #     print('Left Finger ArUco Marker Detection: FAILED')

            # f = fingertips.get('right', None)
            # if f is not None:
            #     fingertip_right_pos = f['pos']
            #     print('Right Finger ArUco Marker Detection: SUCCEEDED')
            # else:
            #     print('Right Finger ArUco Marker Detection: FAILED')
                
            # if (fingertip_left_pos is not None) and (fingertip_right_pos is not None): 
            #     between_fingertips = (fingertip_left_pos + fingertip_right_pos)/2.0
            #     prev_distance_between_fingertips = distance_between_fingertips
            #     distance_between_fingertips = np.linalg.norm(fingertip_left_pos - fingertip_right_pos)
            # elif toy_target is not None:
            #     # The target is so close to the camera that the finger
            #     # markers might be occluded, so hallucinate the between
            #     # fingers position to enhance retraction performance.
            #     if toy_target[2] < max_target_z_for_default_fingertips:
            #         between_fingertips = default_between_fingertips
            #         distance_between_fingertips = prev_distance_between_fingertips

            joint_state = controller.get_joint_state()
            # convert base odometry angle to be in the range -pi to pi
            joint_state['base_odom_theta'] = hm.angle_diff_rad(joint_state['base_odom_theta'], 0.0)

            print('gripper effort = {:.2f}'.format(joint_state['gripper_eff']))

            # if distance_between_fingertips is not None: 
            #     print('distance_between_fingertips = {:.2f} cm'.format(100.0 * distance_between_fingertips))

            if toy_target is not None:
                frames_since_target_detected = 0
            else:
                frames_since_target_detected = frames_since_target_detected + 1

            # if between_fingertips is not None:
            #     frames_since_fingers_detected = 0
            # else: 
            #     frames_since_fingers_detected = frames_since_fingers_detected + 1

            # if (between_fingertips is not None) and (toy_target is not None):
            if toy_target is not None:       
                position_error = toy_target # - between_fingertips
                target_error = np.linalg.norm(position_error)
                last_target_error = target_error  # Track for lock behavior
                print('target_error = {:.2f} cm'.format(100.0 * target_error))

            print('behavior =', behavior)
            print('pre_reach =', pre_reach)
                        
            # if behavior == 'reach':
            prev_behavior = behavior

            if pre_reach:
                cmd = {}

                gripper_ready = False
                if joint_state['gripper_pos'] >= (0.9 * max_joint_state['gripper_pos']):
                    gripper_ready = True
                    cmd['gripper_open'] = 0.0
                else:
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

            # elif (between_fingertips is not None) and (toy_target is not None) and (target_error <= max_distance_for_attempted_reach):
            elif toy_target is not None:            

                x_error, y_error, z_error = position_error

                # Keep wrist yaw stable at 0 degrees instead of servoing
                yaw_velocity = 0.0 - joint_state['wrist_yaw_pos']
                
                target_pitch = joint_state_center['wrist_pitch_pos']
                pitch_velocity = target_pitch - joint_state['wrist_pitch_pos']

                # Keep wrist roll stable at 0 degrees during approach
                target_roll = 0.0
                roll_velocity = target_roll - joint_state['wrist_roll_pos']

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

                base_movement = 0.0
                vel_scale = 10
                # TODO: adjust x_error threshold value
                if (abs(x_error) > 0.005):
                    base_movement = vel_scale * -x_error
                    np.clip(base_movement, -0.1, 0.1)

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
                    'lift_up' : 0.0,  # Disabled to keep tags in view
                    'arm_out' : arm_velocity,
                    'wrist_yaw_counterclockwise' : yaw_velocity,
                    'wrist_pitch_up' : pitch_velocity,
                    'wrist_roll_counterclockwise' : roll_velocity,
                    'base_counterclockwise' : base_rotational_velocity
                }

                if target_error < grasp_if_error_below_this:
                    # Close enough - transition to lock behavior
                    cmd = zero_vel.copy()
                    controller.set_command(cmd)
                    print('Target reached - transitioning to LOCK behavior')
                    behavior = 'lock'
                else:
                    cmd['gripper_open'] = 0.0

                    cmd = {k: overall_visual_servoing_velocity_scale * v for (k,v) in cmd.items()}
                    cmd = {k: joint_visual_servoing_velocity_scale[k] * v for (k,v) in cmd.items()}

                    if motion_on:
                        cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                        cmd['base_forward'] = base_movement
                        controller.set_command(cmd)

            else:
                joint_state = controller.get_joint_state()
                stop_joints = zero_vel.copy()

                # Check if we should transition to lock behavior
                # If we lost detection but were close enough, proceed to lock the button
                if (last_target_error is not None and
                    last_target_error < lock_error_threshold and
                    frames_since_target_detected >= stop_if_target_not_detected_this_many_frames):
                    print('Target lost but was close enough - transitioning to LOCK behavior')
                    behavior = 'lock'
                    cmd = zero_vel.copy()
                elif frames_since_target_detected >= stop_if_target_not_detected_this_many_frames:
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

            # elif behavior == 'lock':
            #     # Lock behavior: rotate CCW 50°, extend arm, hold 5s, rotate CW 100°, restore
            #     if prev_behavior != 'lock':
            #         lock_state_count = 0
            #         lock_phase = 'rotating_ccw'
            #         initial_arm_pos = None
            #         lock_hold_count = 0
            #         print('LOCK: Target reached! Rotating counter-clockwise 50 degrees...')
            #     prev_behavior = behavior

            #     # Safety check - ensure lock_phase is defined
            #     if 'lock_phase' not in locals():
            #         print('WARNING: lock_phase not defined, initializing...')
            #         lock_phase = 'rotating_ccw'
            #         lock_state_count = 0
            #         initial_arm_pos = None
            #         lock_hold_count = 0

            #     print(f'LOCK: Phase = {lock_phase}, Count = {lock_state_count}, wrist_roll = {joint_state["wrist_roll_pos"]:.3f} rad ({np.degrees(joint_state["wrist_roll_pos"]):.1f} deg)')

            #     hold_duration = 30  # 2 seconds at 15Hz
            #     lock_roll_gain = 0.8
            #     lock_roll_max_vel = 0.4
            #     lock_target_pitch = joint_state_center['wrist_pitch_pos']
            #     pitch_hold_vel = lock_target_pitch - joint_state['wrist_pitch_pos']
                
            #     if lock_phase == 'rotating_ccw':
            #         target_roll = -0.785 * mop  # -45 degrees
            #         roll_error = target_roll - joint_state['wrist_roll_pos']
                    
            #         if abs(roll_error) < 0.05:
            #             print('LOCK: CCW rotation complete! Extending arm...')
            #             lock_phase = 'extending'
            #             lock_state_count = 0
            #             initial_arm_pos = joint_state['arm_pos']
            #             cmd = zero_vel.copy()
            #             cmd['wrist_pitch_up'] = pitch_hold_vel
            #             controller.set_command(cmd)
            #         else:
            #             cmd = zero_vel.copy()
            #             cmd['wrist_roll_counterclockwise'] = np.clip(roll_error * lock_roll_gain, -lock_roll_max_vel, lock_roll_max_vel)
            #             cmd['wrist_pitch_up'] = pitch_hold_vel
                        
            #             if motion_on:
            #                 cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
            #                 cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
            #                 controller.set_command(cmd)
                
            #     elif lock_phase == 'extending':
            #         if initial_arm_pos is None:
            #             initial_arm_pos = joint_state['arm_pos']
                    
            #         target_arm_pos = initial_arm_pos + 0.05
            #         arm_error = target_arm_pos - joint_state['arm_pos']
                    
            #         if abs(arm_error) < 0.005:
            #             print('LOCK: Arm extension complete! Holding for 5 seconds...')
            #             lock_phase = 'holding'
            #             lock_state_count = 0
            #             lock_hold_count = 0
            #             cmd = zero_vel.copy()
            #             cmd['wrist_pitch_up'] = pitch_hold_vel
            #             controller.set_command(cmd)
            #         else:
            #             cmd = zero_vel.copy()
            #             cmd['arm_out'] = np.clip(arm_error * 3.0, -1.0, 1.0)
            #             cmd['wrist_pitch_up'] = pitch_hold_vel
                        
            #             if motion_on:
            #                 cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
            #                 cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
            #                 controller.set_command(cmd)
                
            #     elif lock_phase == 'holding':
            #         if lock_hold_count < hold_duration:
            #             cmd = zero_vel.copy()
            #             cmd['wrist_pitch_up'] = pitch_hold_vel
            #             controller.set_command(cmd)
                        
            #             if lock_hold_count % 15 == 0:
            #                 seconds_remaining = (hold_duration - lock_hold_count) // 15
            #                 print(f'LOCK: Holding... {seconds_remaining} seconds remaining')
                        
            #             lock_hold_count += 1
            #         else:
            #             lock_phase = 'rotating_cw'
            #             lock_state_count = 0
            #             print('LOCK: Hold complete! Rotating wrist clockwise to +45 degrees...')
                
            #     elif lock_phase == 'rotating_cw':
            #         target_roll = 0.950 * mop  # +45 degrees
            #         roll_error = target_roll - joint_state['wrist_roll_pos']
                    
            #         if abs(roll_error) < 0.1:
            #             print('LOCK: CW rotation complete! Stopping controller and retracting...')
            #             cmd = zero_vel.copy()
            #             controller.set_command(cmd)
            #             time.sleep(0.3)
                        
            #             controller.stop()
            #             time.sleep(0.2)
                        
            #             print('LOCK: Retracting arm...')
            #             robot.arm.move_to(0.01)
            #             robot.lift.move_to(1.03)
            #             robot.push_command()
            #             robot.wait_command()
                        
            #             print('LOCK: Restoring full pose...')
            #             recenter_robot(robot)
            #             print('LOCK: Pose restored! Exiting program...')
            #             break
            #         else:
            #             cmd = zero_vel.copy()
            #             cmd['wrist_roll_counterclockwise'] = np.clip(roll_error * lock_roll_gain, -lock_roll_max_vel, lock_roll_max_vel)
            #             cmd['wrist_pitch_up'] = pitch_hold_vel
                        
            #             if motion_on:
            #                 cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
            #                 cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
            #                 controller.set_command(cmd)

            #     # Timeout safety (extended to account for all phases)
            #     if lock_state_count > 300:  # ~20 seconds timeout at 15Hz
            #         cmd = zero_vel.copy()
            #         controller.set_command(cmd)
            #         print('LOCK: Timeout! Exiting program...')
            #         break

            #     lock_state_count = lock_state_count + 1

            if toy_target is not None:
                # draw blue circle for the button target position
                draw_origin(image, camera_info, toy_target, (255, 0, 0))
                x,y,z = toy_target * 100.0
                width = target_width_m
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
        prog='Stretch 3 Dial Twisting Demo',
        description='This application demonstrates dial twisting using visual servoing with ArUco markers (tags 23 & 24) and the gripper-mounted D405.',
    )
    parser.add_argument('-e', '--exposure', action='store', type=str, default='low', help=f'Set the D405 exposure to {dh.exposure_keywords} or an integer in the range {dh.exposure_range}') 
    parser.add_argument('--mop', type=str, choices=['open', 'close'], default='open', help='open or close the machine' )
    
    args = parser.parse_args()
    exposure = args.exposure
    mop = 1
    if args.mop == 'close':
        mop = -1

    if not dh.exposure_argument_is_valid(exposure):
        raise argparse.ArgumentTypeError(f'The provided exposure setting, {exposure}, is not a valide keyword, {dh.exposure_keywords}, or is outside of the allowed numeric range, {dh.exposure_range}.')    
    
    main(exposure, mop)
