#!/usr/bin/env python3
"""
Base alignment using ArUco markers detected by head camera.
Aligns robot base tags (#130, #131) with ground tags (#9, #10) in a straight line.
"""

# 9 on the left, 10 on the right


import numpy as np
import cv2
import time
import stretch_body.robot as rb
import d435_helpers as dh
import aruco_detector as ad
import yaml
from yaml.loader import SafeLoader


# Marker IDs
BASE_MARKER_LEFT = 130   # Left marker on robot base
BASE_MARKER_RIGHT = 131  # Right marker on robot base
GROUND_MARKER_LEFT = 9   # Left ground marker
GROUND_MARKER_RIGHT = 10 # Right ground marker

# Marker configuration - all tags are 5cm x 5cm
MARKER_INFO = {
    'default': {
        'length_mm': 50,
        'use_rgb_only': False,
        'name': 'default_marker'
    },
    '9': {
        'length_mm': 50,
        'use_rgb_only': False,
        'name': 'ground_left'
    },
    '10': {
        'length_mm': 50,
        'use_rgb_only': False,
        'name': 'ground_right'
    },
    '130': {
        'length_mm': 50,
        'use_rgb_only': False,
        'name': 'base_left'
    },
    '131': {
        'length_mm': 50,
        'use_rgb_only': False,
        'name': 'base_right'
    }
}


class BaseAligner:
    def __init__(self, robot):
        """
        Args:
            robot: stretch_body Robot instance
        """
        self.robot = robot

        # Alignment thresholds
        self.position_threshold = 0.015  # 1.5cm
        self.angle_threshold = np.deg2rad(1.5)  # 1.5 degrees

        # Movement limits per iteration (for safety)
        self.max_translation = 0.15  # 15cm max per move
        self.max_rotation = np.deg2rad(10)  # 10 degrees max per move

    def setup_head_camera_down(self):
        """Point head camera straight down to see ground markers."""
        print("Moving head to look down...")
        pan = 0.0
        tilt = -np.pi / 2

        self.robot.head.move_to('head_pan', pan)
        self.robot.head.move_to('head_tilt', tilt)
        self.robot.push_command()
        self.robot.wait_command()
        time.sleep(0.5)

    def detect_markers(self, pipeline, aruco_detector, num_samples=5):
        """
        Detect all 4 markers and average positions over multiple frames.

        Returns:
            dict with marker positions or None if detection failed
        """
        detections = {9: [], 10: [], 130: [], 131: []}
        required_markers = [GROUND_MARKER_LEFT, GROUND_MARKER_RIGHT,
                          BASE_MARKER_LEFT, BASE_MARKER_RIGHT]

        for _ in range(num_samples * 3):  # Try more frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            rgb_image = np.asanyarray(color_frame.get_data())
            camera_info = dh.get_camera_info(color_frame)

            aruco_detector.update(rgb_image, camera_info)
            markers = aruco_detector.get_detected_marker_dict()

            for marker_id in required_markers:
                if marker_id in markers:
                    pos = markers[marker_id]['pos']
                    detections[marker_id].append(pos)

            # Check if we have enough samples
            all_have_samples = all(len(detections[m]) >= num_samples
                                   for m in required_markers)
            if all_have_samples:
                break

            time.sleep(0.05)

        # Check if all markers were detected
        for marker_id in required_markers:
            if len(detections[marker_id]) == 0:
                print(f"ERROR: Marker #{marker_id} not detected!")
                return None

        # Average the positions
        result = {}
        for marker_id in required_markers:
            positions = np.array(detections[marker_id])
            result[marker_id] = np.mean(positions, axis=0)
            print(f"Marker #{marker_id}: samples={len(detections[marker_id])}, "
                  f"pos=({result[marker_id][0]:.3f}, {result[marker_id][1]:.3f}, {result[marker_id][2]:.3f})")

        return result

    def compute_alignment_error(self, markers):
        """
        Compute the alignment error between base tags and ground tags.

        Goal: All 4 markers should be on the same line.

        Returns:
            lateral_error: How far the base center is from the ground line (meters)
            angle_error: Angle difference between base line and ground line (radians)
        """
        # Get positions in camera frame (x, y are horizontal, z is depth)
        # When camera looks down: x = forward/back, y = left/right, z = depth

        ground_left = markers[GROUND_MARKER_LEFT][:2]   # x, y only
        ground_right = markers[GROUND_MARKER_RIGHT][:2]
        base_left = markers[BASE_MARKER_LEFT][:2]
        base_right = markers[BASE_MARKER_RIGHT][:2]

        print(f"\n=== DEBUG: Raw positions (x, y) ===")
        print(f"  Ground #9 (left):  ({ground_left[0]:.3f}, {ground_left[1]:.3f})")
        print(f"  Ground #10 (right): ({ground_right[0]:.3f}, {ground_right[1]:.3f})")
        print(f"  Base #130 (left):  ({base_left[0]:.3f}, {base_left[1]:.3f})")
        print(f"  Base #131 (right): ({base_right[0]:.3f}, {base_right[1]:.3f})")

        # Ground line direction and center
        ground_vec = ground_right - ground_left
        ground_angle = np.arctan2(ground_vec[1], ground_vec[0])
        ground_center = (ground_left + ground_right) / 2

        # Base line direction and center
        base_vec = base_right - base_left
        base_angle = np.arctan2(base_vec[1], base_vec[0])
        base_center = (base_left + base_right) / 2

        print(f"\n=== DEBUG: Vectors and angles ===")
        print(f"  Ground vec: ({ground_vec[0]:.3f}, {ground_vec[1]:.3f})")
        print(f"  Ground angle: {np.rad2deg(ground_angle):.1f}°")
        print(f"  Base vec: ({base_vec[0]:.3f}, {base_vec[1]:.3f})")
        print(f"  Base angle: {np.rad2deg(base_angle):.1f}°")

        # Angle error: difference between base line and ground line angles
        angle_error = base_angle - ground_angle
        # Normalize to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        # Lateral error: perpendicular distance from base center to ground line
        # Vector from ground_center to base_center (simplified)
        to_base = base_center - ground_center

        # Ground line unit vector
        ground_unit = ground_vec / np.linalg.norm(ground_vec)

        # Perpendicular component (cross product in 2D)
        # Positive = base is to the left of ground line direction
        lateral_error = to_base[0] * ground_unit[1] - to_base[1] * ground_unit[0]

        # Longitudinal error: component along ground line direction
        # This tells us if base center is ahead or behind ground center
        # Positive = base needs to move forward (along camera x-axis)
        longitudinal_error = -to_base[0]  # Simplified: just use x difference

        print(f"\n=== DEBUG: Errors ===")
        print(f"  to_base (center diff): ({to_base[0]:.3f}, {to_base[1]:.3f})")
        print(f"  Angle error: {np.rad2deg(angle_error):.2f}°")
        print(f"  Lateral error: {lateral_error * 100:.1f} cm")
        print(f"  Longitudinal error: {longitudinal_error * 100:.1f} cm")

        return lateral_error, longitudinal_error, angle_error

    def compute_correction(self, lateral_error, longitudinal_error, angle_error):
        """
        Compute robot movements to correct alignment.

        When robot rotates by θ, ground tags rotate by -θ in camera frame.
        So to reduce angle_error, we rotate BY angle_error (not negative).

        Returns:
            rotation: base rotation in radians (positive = CCW)
            translation: base translation in meters (positive = forward)
            lateral: lateral translation in meters (positive = left)
        """
        # Rotation correction: rotate BY the angle error to align
        # When angle_error > 0 (ground line tilted more), rotate positive (CCW)
        rotation = angle_error  # FIXED: was -angle_error (wrong direction!)

        # Translation: move forward to reduce longitudinal error
        # longitudinal_error < 0 means robot needs to move forward
        translation = -longitudinal_error  # This was correct, keep it

        # Lateral error (for reference only, not corrected automatically)
        lateral = lateral_error

        # Clamp to safe limits
        rotation = np.clip(rotation, -self.max_rotation, self.max_rotation)
        translation = np.clip(translation, -self.max_translation, self.max_translation)

        return rotation, translation, lateral

    def execute_correction(self, rotation, translation, lateral):
        """Execute the correction movements."""
        print(f"\n=== Proposed correction ===")
        print(f"  Rotation: {np.rad2deg(rotation):.2f}°")
        print(f"  Translation: {translation * 100:.1f} cm")
        print(f"  Lateral: {lateral * 100:.1f} cm")

        # Ask for confirmation before moving
        response = input("\nExecute this correction? [y/n/q]: ").strip().lower()
        if response == 'q':
            raise KeyboardInterrupt("User quit")
        if response != 'y':
            print("Skipping correction...")
            return

        # First correct rotation (small amounts only)
        if abs(rotation) > self.angle_threshold / 2:
            print(f"  -> Rotating {np.rad2deg(rotation):.2f}°")
            self.robot.base.rotate_by(rotation)
            self.robot.push_command()
            self.robot.wait_command()
            time.sleep(0.3)

        # Then correct translation (forward/back)
        if abs(translation) > self.position_threshold / 2:
            print(f"  -> Translating {translation * 100:.1f} cm")
            self.robot.base.translate_by(translation, v_m=0.05)
            self.robot.push_command()
            self.robot.wait_command()
            time.sleep(0.3)

        # Skip lateral correction for now - it's complex and error-prone
        # The rotation + translation should handle most cases
        if abs(lateral) > self.position_threshold * 2:
            print(f"  Note: Lateral error of {lateral * 100:.1f} cm - may need manual adjustment")

    def align(self, pipeline, aruco_detector, max_iterations=10):
        """
        Main alignment loop.

        Returns:
            True if alignment successful, False otherwise
        """
        print("\n" + "=" * 50)
        print("Starting Base Alignment")
        print("Goal: Align base tags (#130, #131) with ground tags (#9, #10)")
        print("=" * 50)

        # Point head down
        self.setup_head_camera_down()
        time.sleep(0.5)

        for iteration in range(max_iterations):
            print(f"\n--- Iteration {iteration + 1}/{max_iterations} ---")

            # Detect all markers
            markers = self.detect_markers(pipeline, aruco_detector)
            if markers is None:
                print("Detection failed, retrying...")
                time.sleep(0.5)
                continue

            # Compute alignment error
            lateral_err, longitudinal_err, angle_err = self.compute_alignment_error(markers)

            # Check if aligned
            if (abs(lateral_err) < self.position_threshold and
                abs(longitudinal_err) < self.position_threshold and
                abs(angle_err) < self.angle_threshold):
                print("\n" + "=" * 50)
                print("ALIGNMENT COMPLETE!")
                print(f"Final errors: lateral={lateral_err*100:.1f}cm, "
                      f"longitudinal={longitudinal_err*100:.1f}cm, "
                      f"angle={np.rad2deg(angle_err):.2f}°")
                print("=" * 50)
                return True

            # Compute and execute correction
            rotation, translation, lateral = self.compute_correction(
                lateral_err, longitudinal_err, angle_err)
            self.execute_correction(rotation, translation, lateral)

            time.sleep(0.3)

        print("\n" + "=" * 50)
        print("ALIGNMENT INCOMPLETE - Max iterations reached")
        print("=" * 50)
        return False


def main():
    """Test alignment functionality."""
    # print("=" * 50)
    # print("Base Alignment Test")
    # print("=" * 50)
    # print("\nRequired markers:")
    # print("  Ground: #9 (left), #10 (right) - 5cm x 5cm")
    # print("  Base: #130 (left), #131 (right)")
    # print("\nPlace ground markers on floor, on left and right sides")
    # print("of where you want the robot to align.")

    # input("\nPress Enter to start...")

    # Initialize robot
    print("\nStarting robot...")
    robot = rb.Robot()
    robot.startup()

    # Start head camera (D435)
    print("Starting D435 head camera...")
    pipeline, profile = dh.start_d435('auto')

    # Initialize ArUco detector
    aruco_detector = ad.ArucoDetector(MARKER_INFO)

    # Create aligner
    aligner = BaseAligner(robot)

    try:
        success = aligner.align(pipeline, aruco_detector)
        if success:
            print("\nAlignment successful!")
        else:
            print("\nAlignment failed or incomplete")
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        pipeline.stop()
        robot.stop()
        print("Done!")


if __name__ == '__main__':
    main()
