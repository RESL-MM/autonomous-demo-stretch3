"""
Complete MVP Demo Script for the Stretch3
Assume it starts at the machine => open machine => navigates to wafer station => picks up wafer
=> navigates to tray => deposits wafer => navigates to machine => closes machine => waits => opens
machine => navigates to tray => withdraws wafer => navigates to wafer station => put down wafer
=> navigate to machine => close machine => demo done

Demo to be extended by starting at finishing at Stretch3 charging dock
"""

import time
import math
import stretch_body.robot as rb
import argparse
import d405_helpers as dh
import d435_helpers as d435h
import twist_and_adjust
import button_and_adjust
import base_alignment
import station_navigation

FROM_MACHINE_TO_WTABLE = 2.1
QUARTER_COUNTERCLOCK = math.pi/2
QUARTER_CLOCK = -QUARTER_COUNTERCLOCK
NEXT_TO_MACHINE_OR_TRAY = 0.80
FROM_WTABLE_TO_TRAY = 1
FROM_TRAY_TO_MACHINE = 1.0

def default_gripper(robot):
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.end_of_arm.move_to('wrist_roll', 0.0)
    robot.end_of_arm.move_to('stretch_gripper', 50) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()

def move(robot, distance: float):
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.base.translate_by(distance, v_m=0.5)
    robot.push_command()
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    print(f"waited for move by {angle}m command? {robot.wait_command()}")

def do_dw_pupd(robot, dw=True, put_down=False):
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


def main():
    parser = argparse.ArgumentParser(
        prog='Stretch 3 Complete Demo',
        description='Runs through a single itertion of a wafer deposit/withdraw loop',
    )

    parser.add_argument(
        '-e', '--exposure', action='store', type=str, default='low',
        help=f'Set the D405 and D435 exposure to {dh.exposure_keywords} or an integer in the range {dh.exposure_range}',
    )

    if not dh.exposure_argument_is_valid(exposure):
        raise argparse.ArgumentTypeError(
            f'The provided exposure setting, {exposure}, is not a valid keyword, '
            f'{dh.exposure_keywords}, or is outside of the allowed numeric range, {dh.exposure_range}.')


    args = parser.parse_args()
    exposure = args.exposure

    robot = None

    try:
        robot = rb.Robot()
        robot.startup()

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        move(0.12)

        print('=== Opening the Machine ===')
        twist_and_adjust.run(robot, op='open')
            
        move(-0.25)

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        print('=== Moving to Push Button ===')
        move(-0.02)

        print('=== Pausing for 1 second ===')
        time.sleep(1.0)

        button_and_adjust.run(robot)

        print('=== Going to Pick Up Wafer ===')
        move(-FROM_MACHINE_TO_WTABLE)
        rotate(QUARTER_CLOCK)

        print('=== Pausing for 1 sec ===')
        time.sleep(1.0)

        print('=== Aligning with Wafer Station ==')
        station_navigation.run(robot, 'wafer_station', horizontal_align=False)

        rotate(QUARTER_COUNTERCLOCK)

        station_navigation.run(robot, 'wafer_station', horizontal_align=True)

        print('=== Picking Up Wafer ===')
        do_dw_pupd(False, False)

        print('=== Moving to Tray ===')
        station_navigation.run(robot, 'tray', horizontal_align=False)

        rotate(QUARTER_COUNTERCLOCK)

        print('=== Aligning with Tray ===')
        station_navigation.run(robot, 'tray', horizontal_align=True)

        print('=== Depositing Wafer ===')
        do_dw_pupd(True, True)

        print('=== Moving to Machine ===')
        move(0.9)
        rotate(QUARTER_CLOCK)
        move(FROM_TRAY_TO_MACHINE)

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        move(0.12)

        print('=== Closing the Machine ===')
        twist_and_adjust.run(robot, op='close')
            
        move(-0.25)

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        print('=== Moving to Push Button ===')
        move(-0.02)

        print('=== Pausing for 1 second ===')
        time.sleep(1.0)

        button_and_adjust.run(robot)

        # HEILUHWUHF
        
        time.sleep(5) # simulate waiting for recipe to run-- TODO: extend by having stretch dock and undock

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        move(0.12)

        print('=== Opening the Machine ===')
        twist_and_adjust.run(robot, op='open')
            
        move(-0.25)

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        print('=== Moving to Push Button ===')
        move(-0.02)

        print('=== Pausing for 1 second ===')
        time.sleep(1.0)

        button_and_adjust.run(robot)


        print('=== Moving to Tray ===')
        move(-FROM_TRAY_TO_MACHINE)
        rotate(QUARTER_COUNTERCLOCK)
        move(-0.9)

        print('=== Aligning with Tray ===')
        station_navigation.run(robot, 'tray', horizontal_align=True)

        print('=== Withdrawing Wafer ===')
        do_dw_pupd(True, False)

        print('=== Moving to Wafer Station ===')
        move(0.9)
        rotate(QUARTER_CLOCK)
        move(-FROM_WTABLE_TO_TRAY)
        rotate(QUARTER_CLOCK)

        print('=== Going to with Wafer Station ==')
        station_navigation.run(robot, 'wafer_station', horizontal_align=False)

        rotate(QUARTER_COUNTERCLOCK)

        print('=== Aligning with Wafer Station ==')
        station_navigation.run(robot, 'wafer_station', horizontal_align=True)

        print('=== Putting Down Wafer ===')
        do_dw_pupd(False, True)

        print('=== Going to Machine ===')
        rotate(QUARTER_COUNTERCLOCK)
        move(0.95)
        rotate(QUARTER_CLOCK)
        move(FROM_MACHINE_TO_WTABLE)

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        move(0.12)

        print('=== Closing the Machine ===')
        twist_and_adjust.run(robot, op='close')
            
        move(-0.25)

        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        print('=== Moving to Push Button ===')
        move(-0.02)

        print('=== Pausing for 1 second ===')
        time.sleep(1.0)

        button_and_adjust.run(robot)

        print('=== Complete Demo Done! ===')
    finally:
        if robot is not None:
            robot.stop()


if __name__ == '__main__':
    main()
