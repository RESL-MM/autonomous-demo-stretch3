"""
Complete MVP Demo Script for the Stretch3
Assume it starts at the machine => open machine => navigates to wafer station => picks up wafer
=> navigates to tray => deposits wafer => navigates to machine => closes machine => waits => opens
machine => navigates to tray => withdraws wafer => navigates to wafer station => put down wafer
=> navigate to machine => close machine => demo done

Demo to be extended by starting at finishing at Stretch3 charging dock
"""

import subprocess
import sys
import time
import math
import stretch_body.robot as rb
import argparse
import d405_helpers as dh
import d435_helpers as d435h

FROM_MACHINE_TO_WTABLE = 2.1
QUARTER_COUNTERCLOCK = math.pi/2
QUARTER_CLOCK = -QUARTER_COUNTERCLOCK
NEXT_TO_MACHINE_OR_TRAY = 0.80
FROM_WTABLE_TO_TRAY = 1
FROM_TRAY_TO_MACHINE = 1.2

def default_gripper():
    robot = rb.Robot()
    robot.startup()
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.end_of_arm.move_to('wrist_roll', 0.0)
    robot.end_of_arm.move_to('stretch_gripper', 50) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()
    robot.stop()

def move(distance: float):
    robot = rb.Robot()
    robot.startup()
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.base.translate_by(distance, v_m=0.5)
    robot.push_command()
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")
    robot.stop()

def rotate(angle: float):
    robot = rb.Robot()
    robot.startup()
    robot.base.rotate_by(angle)
    robot.push_command()
    print(f"waited for move by {angle}m command? {robot.wait_command()}")
    robot.stop()

def do_dw_pupd(dw=True, put_down=False):
    STOW_ANGLE = (3 * math.pi)/4
    PUPD_DIST = 0.03

    # deposit/withdraw values
    DW_HEIGHT = 1.05
    DW_LENGTH = 0.35
    PUPD_DIST = 0.02

    if not dw:
        # pick up/put down values
        DW_HEIGHT = 0.98
        DW_LENGTH = 0.1
        PUPD_DIST = 0.08

    print(f"{DW_HEIGHT}, {DW_LENGTH}, {PUPD_DIST}")

    robot = rb.Robot()
    robot.startup()
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.lift.move_to(DW_HEIGHT)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', 0.0)
    time.sleep(2)

    robot.arm.move_to(DW_LENGTH)
    robot.push_command()
    robot.wait_command()
    
    robot.lift.move_by(-PUPD_DIST, v_m=0.02)
    robot.push_command()
    robot.wait_command()

    if put_down:
        robot.end_of_arm.move_to('stretch_gripper', -70) # [-100, 100] => [fully closed, fully open]
        robot.push_command()
        robot.wait_command()
        time.sleep(1)
        robot.end_of_arm.move_to('stretch_gripper', 50) # [-100, 100] => [fully closed, fully open]
        robot.push_command()
        robot.wait_command()

    robot.lift.move_by(PUPD_DIST)
    robot.push_command()
    robot.wait_command()

    robot.arm.move_to(0.0)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)

    robot.stop()


def main():
    parser = argparse.ArgumentParser(
        prog='Stretch 3 Complete Demo',
        description='Runs through a single itertion of a wafer deposit/withdraw loop',
    )

    parser.add_argument(
        '-e', '--exposure', action='store', type=str, default='low',
        help=f'Set the D405 and D435 exposure to {dh.exposure_keywords} or an integer in the range {dh.exposure_range}',
    )
    args = parser.parse_args()
    exposure = args.exposure

    if not dh.exposure_argument_is_valid(exposure):
        raise argparse.ArgumentTypeError(
            f'The provided exposure setting, {exposure}, is not a valid keyword, '
            f'{dh.exposure_keywords}, or is outside of the allowed numeric range, {dh.exposure_range}.'
        )

    print('=== Opening the Machine ===')
    result = subprocess.run(
        [sys.executable, 'twist_and_adjust.py', '-e', exposure, '--mop', 'open'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Dial twisting demo exited with code {result.returncode}')

    print('=== Moving to Push Button ===')
    move(-0.095)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    result = subprocess.run(
        [sys.executable, 'button_pressing_demo.py', '-e', exposure],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Button pressing demo exited with code {result.returncode}')

    print('=== Going to Pick Up Wafer ===')
    move(-FROM_MACHINE_TO_WTABLE)
    rotate(QUARTER_CLOCK)

    print('=== Pausing for 1 sec ===')
    time.sleep(1.0)

    print('=== Aligning with Wafer Station ==')
    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'wafer_station'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    rotate(QUARTER_COUNTERCLOCK)

    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'wafer_station', '--horizontal'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    print('=== Picking Up Wafer ===')
    do_dw_pupd(False, False)

    print('=== Moving to Tray ===')
    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'tray'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    rotate(QUARTER_COUNTERCLOCK)

    print('=== Aligning with Tray ===')
    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'tray', '--horizontal'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    print('=== Depositing Wafer ===')
    do_dw_pupd(True, True)

    print('=== Moving to Machine ===')
    move(0.9)
    rotate(QUARTER_CLOCK)
    move(FROM_TRAY_TO_MACHINE)

    print('=== Closing Machine ===')
    result = subprocess.run(
        [sys.executable, 'twist_and_adjust.py', '-e', exposure, '--mop', 'close'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Dial twisting demo exited with code {result.returncode}')

    print('=== Moving to Push Button ===')
    move(-0.095)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    result = subprocess.run(
        [sys.executable, 'button_pressing_demo.py', '-e', exposure],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Button pressing demo exited with code {result.returncode}')

    
    time.sleep(5) # simulate waiting for recipe to run-- TODO: extend by having stretch dock and undock

    print('=== Opening the Machine ===')
    result = subprocess.run(
        [sys.executable, 'twist_and_adjust.py', '-e', exposure, '--mop', 'open'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Dial twisting demo exited with code {result.returncode}')

    result = subprocess.run(
        [sys.executable, 'button_pressing_demo.py', '-e', exposure],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Button pressing demo exited with code {result.returncode}')


    print('=== Moving to Tray ===')
    move(-FROM_TRAY_TO_MACHINE)
    rotate(QUARTER_COUNTERCLOCK)
    move(-0.9)

    print('=== Aligning with Tray ===')
    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'tray', '--horizontal'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    print('=== Withdrawing Wafer ===')
    do_dw_pupd(True, False)

    print('=== Moving to Wafer Station ===')
    move(0.9)
    rotate(QUARTER_CLOCK)
    move(-FROM_WTABLE_TO_TRAY)
    rotate(QUARTER_CLOCK)

    print('=== Going to with Wafer Station ==')
    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'wafer_station'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    rotate(QUARTER_COUNTERCLOCK)

    print('=== Aligning with Wafer Station ==')
    result = subprocess.run(
        [sys.executable, 'station_navigation.py', '-e', exposure, '--tag_name', 'wafer_station', '--horizontal'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Wafer station alignment exited with code {result.returncode}')

    print('=== Putting Down Wafer ===')
    do_dw_pupd(False, True)

    print('=== Going to Machine ===')
    rotate(QUARTER_COUNTERCLOCK)
    move(0.95)
    rotate(QUARTER_CLOCK)
    move(FROM_MACHINE_TO_WTABLE)

    print('=== Closing Machine ===')
    result = subprocess.run(
        [sys.executable, 'twist_and_adjust.py', '-e', exposure, '--mop', 'close'],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Dial twisting demo exited with code {result.returncode}')

    print('=== Moving to Push Button ===')
    move(-0.095)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    result = subprocess.run(
        [sys.executable, 'button_pressing_demo.py', '-e', exposure],
        cwd=sys.path[0] or '.',
    )
    if result.returncode != 0:
        print(f'Button pressing demo exited with code {result.returncode}')

    print('=== Complete Demo Done! ===')


if __name__ == '__main__':
    main()
