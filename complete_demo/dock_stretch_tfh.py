import d435_helpers as dh
import pyrealsense2 as rs
import numpy as np
import cv2
import normalized_velocity_control as nvc
import stretch_body.robot as rb
import aruco_detector as ad
import yaml
import s3_tf_helper as tfh
from yaml.loader import SafeLoader
from scipy.spatial.transform import Rotation
from hello_helpers import hello_misc as hm
import loop_timer as lt
from stretch_body import robot_params
from stretch_body import hello_utils as hu

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
    'gripper_pos' : .0 #3.5 #4.0 #3.0 
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
    
    robot.end_of_arm.get_joint('wrist_yaw').move_to(3 * np.pi/4)
    robot.end_of_arm.get_joint('wrist_pitch').move_to(0.0)
    robot.push_command()
    robot.wait_command()
    
    robot.end_of_arm.get_joint('wrist_roll').move_to(joint_state_center['wrist_roll_pos'])
    robot.push_command()
    robot.wait_command()
    
    robot.lift.move_to(0.9)
    robot.push_command()
    robot.wait_command()

    # robot.end_of_arm.get_joint('stretch_gripper').move_to(joint_state_center['gripper_pos'])
    # robot.push_command()
    # robot.wait_command()
        

def run(robot, exposure='low'):
    controller = None
    pipeline = None

    try:
        recenter_robot(robot)
        controller = nvc.NormalizedVelocityControl(robot)
        controller.reset_base_odometry()

        robot.head.move_to('head_pan', -np.pi/2)
        robot.head.move_to('head_tilt', 0.0)

        marker_info = {}
        with open('aruco_marker_info.yaml') as f:
            marker_info = yaml.load(f, Loader=SafeLoader)

        detect_aruco_button_on = True
        aruco_detector = ad.ArucoDetector(marker_info=marker_info, show_debug_images=True, use_apriltag_refinement=False, brighten_images=True)
        # aruco_to_fingertips = af.ArucoToFingertips(default_height_above_mounting_surface=af.suctioncup_height['cup_bottom'])

        head_nav_info = tfh.CameraInfo
        head_nav_info.cam_name = 'head_nav'
        # TODO: update offset and later add better way of updating (e.g. via yaml)
        head_nav_info.fixed_pos_offset = np.array([0.0, 0.08, 1.13])
        
        transform_helper = tfh.TransformHelper(robot)
        transform_helper.add_camera(head_nav_info)

        first_frame = True

        behavior = 'reach'
        prev_behavior = 'reach'
        pre_reach = True
        last_target_error = None  # Track the last known target error before detection was lost

        # Assume that the gripper starts out fully opened
        distance_between_fingertips = distance_between_fully_open_fingertips
        prev_distance_between_fingertips = distance_between_fully_open_fingertips

        pipeline, profile = dh.start_d435(exposure)

        frames_since_target_detected = 0
        # frames_since_fingers_detected = 0
            
        loop_timer = lt.LoopTimer()

        fingertips = {}

        DEBUG_TIME = 120
        debug_timer = 0
        
        while True:
            loop_timer.start_of_iteration()

            wafer_station = None
            wafer_station_normal = None

            # fingertip_left_pos = None       
            # fingertip_right_pos = None
            # between_fingertips = None
            # distance_between_fingertips = None
            
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
                # fingertips = aruco_to_fingertips.get_fingertips(markers)                    
                                                                                                                               
                for k in markers:                                                           
                    m = markers[k]                                                          
                    name = m['info']['name']

                    if name == 'docking_station':
                        wafer_station = m['pos']  
                        wafer_station_normal = m['z_axis']     

                if wafer_station is not None:
                    # assigns position and accounts for the orientation of the d435i
                    wafer_station = np.array([wafer_station[1], -wafer_station[0], wafer_station[2]])
                    wafer_station_normal = np.array([wafer_station_normal[1], -wafer_station_normal[0], wafer_station_normal[2]])
                    wafer_station_normal = wafer_station_normal / np.linalg.norm(wafer_station_normal)                 
                                                                                         
            target_name = 'docking_station'
            if wafer_station is None and debug_timer >= DEBUG_TIME:
                print(target_name + ' Detection: FAILED')
            elif debug_timer >= DEBUG_TIME:
                print(target_name + ' Detection: SUCCEEDED')

            joint_state = controller.get_joint_state()
            # convert base odometry angle to be in the range -pi to pi
            joint_state['base_odom_theta'] = hm.angle_diff_rad(joint_state['base_odom_theta'], 0.0)

            if wafer_station is not None:
                frames_since_target_detected = 0
            else:
                frames_since_target_detected = frames_since_target_detected + 1

            if wafer_station is not None:         
                position_error = wafer_station
                cam_roll = 0
                cam_pitch = -joint_state['head_tilt_pos']
                cam_yaw = -joint_state['head_pan_pos']
                if debug_timer >= DEBUG_TIME:
                    print(f"cam pos error: {position_error}")
                    print(f"cam rotation: {np.array([cam_roll, cam_pitch, cam_yaw])}")
                cam_x, cam_y, cam_z = position_error[0], position_error[1], position_error[2]
                position_error = transform_helper.get_base_coord_from_cam_coord('head_nav',
                                                                                cam_x, cam_y, cam_z,
                                                                                0, 0, 0,
                                                                                cam_roll, cam_pitch, cam_yaw)

                # TODO: check and implement the CAMERA FRAME processing-- implement transform here?

                target_error = np.linalg.norm(position_error)
                rotation_error = np.arctan2(-wafer_station_normal[0], -wafer_station_normal[2])
                # print('target_error = {:.2f} cm'.format(100.0 * target_error))

            # print('behavior =', behavior)
            # print('pre_reach =', pre_reach)
                        
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
            elif wafer_station is not None:           
                x_error, y_error, z_error = position_error[0], position_error[1], position_error[2] # TODO: check-- this SHOULD be in base frame

                if debug_timer >= DEBUG_TIME:
                    print(f"pos error:     {position_error}")
                    print(f"target error: {target_error}")
                    print(f"rot error: {rotation_error}")
                    print("\n\n")

                # k_face = 1.5
                # k_base = 5.0
                # max_rotation = np.pi/12
                # rotation_tolerance = 0.01 # radians
                # alignment_tolerance = 0.01 # meters
                # station_tolerance = 0.9

                # base_rotational_vel = np.clip(k_face * rotation_error, -max_rotation, max_rotation)
                # base_movement = np.clip(k_base * x_error, -1.0, 1.0)
                # base_station_movement = np.clip(k_base * z_error, -1.0, 1.0)

                # reached = True

                # cmd = zero_vel.copy()

                # #TODO: make error correction more robust (large distance and angle correction case)

                # if (abs(rotation_error) > rotation_tolerance):
                #     cmd['base_counterclockwise'] = base_rotational_vel
                #     print('Aligning with Station')
                #     reached = False
                
                # if (abs(x_error) > alignment_tolerance):
                #     cmd['base_forward'] = base_movement
                #     print('Horizontally Aligning with Station')
                #     reached = False

                # if (abs(z_error) > station_tolerance):
                #     cmd['base_forward'] = base_station_movement
                #     print('Moving to Station')
                #     reached = False

                # if reached:
                #     break

                # controller.set_command(cmd)

            else:
                joint_state = controller.get_joint_state()
                stop_joints = zero_vel.copy()

                if frames_since_target_detected >= stop_if_target_not_detected_this_many_frames:
                    cmd = stop_joints
                else:
                    # Stop at Boundaries
                    cmd = { k:v for (k,v) in stop_joints.items() if (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]]) }
                    cmd = { k:v for (k,v) in stop_joints.items() if (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]]) }

                if cmd:
                    cmd = { k: ( 0.0 if ((v < 0.0) and (joint_state[vel_cmd_to_pos[k]] < min_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                    cmd = { k: ( 0.0 if ((v > 0.0) and (joint_state[vel_cmd_to_pos[k]] > max_joint_state[vel_cmd_to_pos[k]])) else v ) for (k,v) in cmd.items()}
                    controller.set_command(cmd)

            loop_timer.end_of_iteration()
            if print_timing and debug_timer >= DEBUG_TIME: 
                loop_timer.pretty_print(minimum=True)
                print("\n\n")

            if debug_timer >= DEBUG_TIME:
                debug_timer = 0
            else:
                debug_timer += 1
                
    finally:
        if controller is not None:
            controller.stop()
        if pipeline is not None:
            pipeline.stop()

def main():
    robot = rb.Robot()

    if not robot.startup():
        print("Failed to start robot")
        return

    try:
        run(robot)
    finally:
        robot.stop()


if __name__ == "__main__":
    main()