#!/usr/bin/env python3
"""
Simple test script to verify head camera can detect ground ArUco markers.
Run this first before testing full alignment.
"""

import numpy as np
import cv2
import time
import stretch_body.robot as rb
import d435_helpers as dh
import aruco_detector as ad


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

# Required markers for alignment
REQUIRED_MARKERS = [9, 10, 130, 131]


def setup_head_look_down(robot):
    """Point head camera straight down."""
    print("Moving head to look down...")

    # Pan: 0 = forward, pi/2 = left
    # Tilt: 0 = level, -pi/2 = straight down
    pan = 0.0
    tilt = -np.pi / 2

    robot.head.move_to('head_pan', pan)
    robot.head.move_to('head_tilt', tilt)
    robot.push_command()
    robot.wait_command()
    time.sleep(1.0)
    print(f"Head positioned: pan={np.rad2deg(pan):.1f}°, tilt={np.rad2deg(tilt):.1f}°")


def main():
    print("=" * 50)
    print("Head Camera ArUco Detection Test")
    print("=" * 50)
    print("\nRequired markers (all 5cm x 5cm):")
    print("  Ground: #9 (left side), #10 (right side)")
    print("  Base: #130, #131 (already on robot)")
    print("\nPlace ground markers on floor where camera can see them.\n")

    # Initialize robot
    print("Starting robot...")
    robot = rb.Robot()
    robot.startup()

    # Point head down
    setup_head_look_down(robot)

    # Start head camera (D435)
    print("\nStarting D435 head camera...")
    pipeline, profile = dh.start_d435('auto')

    # Initialize ArUco detector
    aruco_detector = ad.ArucoDetector(MARKER_INFO)

    print("\n" + "=" * 50)
    print("Scanning for ArUco markers... (Press Ctrl+C to stop)")
    print("=" * 50 + "\n")

    try:
        frame_count = 0
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame:
                continue

            rgb_image = np.asanyarray(color_frame.get_data())
            camera_info = dh.get_camera_info(color_frame)

            # Detect markers
            aruco_detector.update(rgb_image, camera_info)
            markers = aruco_detector.get_detected_marker_dict()

            frame_count += 1

            # Draw detected markers on image
            display_image = aruco_detector.aruco_marker_collection.draw_markers(rgb_image.copy())

            if markers:
                # Check which required markers are detected
                detected_required = [m for m in REQUIRED_MARKERS if m in markers]
                missing = [m for m in REQUIRED_MARKERS if m not in markers]

                if frame_count % 10 == 0:  # Print every 10 frames
                    print(f"\n[Frame {frame_count}] Detected {len(markers)} marker(s):")
                    for marker_id, data in markers.items():
                        pos = data['pos']
                        marker_type = "BASE" if marker_id in [130, 131] else "GROUND" if marker_id in [9, 10] else "OTHER"
                        print(f"  #{marker_id} ({marker_type}): x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}m")

                    if missing:
                        print(f"  MISSING: {missing}")
                    else:
                        print(f"  ✓ All 4 required markers detected!")

                # Draw info on image
                y_offset = 30
                for marker_id in REQUIRED_MARKERS:
                    if marker_id in markers:
                        pos = markers[marker_id]['pos']
                        color = (0, 255, 0)  # Green
                        text = f"#{marker_id}: z={pos[2]:.2f}m"
                    else:
                        color = (0, 0, 255)  # Red
                        text = f"#{marker_id}: MISSING"
                    cv2.putText(display_image, text, (10, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    y_offset += 25
            else:
                if frame_count % 30 == 0:
                    print(f"[Frame {frame_count}] No markers detected")

            # Show image
            cv2.imshow('Head Camera - ArUco Detection', display_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        robot.stop()
        print("Done!")


if __name__ == '__main__':
    main()
