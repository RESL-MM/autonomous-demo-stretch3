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

FROM_MACHINE_TO_WTABLE = 2
QUARTER_COUNTERCLOCK = math.pi/2
QUARTER_CLOCK = -QUARTER_COUNTERCLOCK
NEXT_TO_MACHINE_OR_TRAY = 0.80
FROM_WTABLE_TO_TRAY = 1
FROM_TRAY_TO_MACHINE = 1.0

DEBUGGING = False

def default_gripper(robot):
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.end_of_arm.move_to('wrist_roll', 0.0)
    robot.end_of_arm.move_to('stretch_gripper', 0) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()

def move(robot, distance: float):
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.base.translate_by(distance, v_m=1.0)
    robot.push_command()
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    print(f"waited for move by {angle}m command? {robot.wait_command()}")

def do_dw_pupd(robot, dw=True, put_down=False):
    # deposit/withdraw values
    DW_HEIGHT = 1.1
    DW_LENGTH = 0.46
    PUPD_DIST = 0.05

    if not dw:
        # pick up/put down values
        DW_HEIGHT = 0.98
        DW_LENGTH = 0.05
        PUPD_DIST = 0.11

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
    
    robot.lift.move_by(-PUPD_DIST, v_m=0.03)
    robot.push_command()
    robot.wait_command()

    if put_down:
        robot.end_of_arm.move_to('stretch_gripper', -70) # [-100, 100] => [fully closed, fully open]
        time.sleep(2.5)

    robot.lift.move_by(PUPD_DIST)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('stretch_gripper', 25) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()

    robot.arm.move_to(0.0)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)

def debug_test(robot):   
    button_op_test(robot) 
    # machine_op_debug(robot)
    # go_to_tray_debug(robot)
    return

def go_to_tray_debug(robot):
    print('=== Moving to Tray ===')
    station_navigation.run(robot, 'tray', horizontal_align=False)

    rotate(robot, QUARTER_COUNTERCLOCK)

    print('=== Aligning with Tray ===')
    station_navigation.run(robot, 'tray', horizontal_align=True)

    print('=== Depositing Wafer ===')
    do_dw_pupd(robot, True, True)
                        

def machine_op_debug(robot):
    #for i in range (0,5):
    print('=== Aligning with Machine ===')
    base_alignment.run(robot)

    move(robot, 0.12)

    print('=== Opening the Machine ===')
    twist_and_adjust.run(robot, op='open')
        
    move(robot, -0.2)

    print('=== Aligning with Machine ===')
    base_alignment.run(robot)

    print('=== Moving to Push Button ===')
    move(robot, -0.02)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    button_and_adjust.run(robot)

    print('=== Aligning with Machine ===')
    base_alignment.run(robot)

    move(robot, 0.12)

    print('=== Closing the Machine ===')
    twist_and_adjust.run(robot, op='close')
        
    move(robot, -0.25)

    print('=== Aligning with Machine ===')
    base_alignment.run(robot)

    print('=== Moving to Push Button ===')
    move(robot, -0.02)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    button_and_adjust.run(robot)

def button_op_test(robot):
    for i in range (0,5):
        print('=== Aligning with Machine ===')
        base_alignment.run(robot)

        print('=== Moving to Push Button ===')
        move(robot, -0.02)

        print('=== Pausing for 1 second ===')
        time.sleep(1.0)

        button_and_adjust.run(robot)
            
        move(robot, -0.15)



def main():
    try:
        robot = rb.Robot()
        robot.startup()
        
        DEBUG = False 

        if DEBUG:
            debug_test(robot)
            return
        
        i = 0

        while i < 1:
            i += 1

            print(f'=== iteration: {i} ===\n\n')

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            move(robot, 0.12)

            print('=== Opening the Machine ===')
            twist_and_adjust.run(robot, op='open')
                
            move(robot, -0.25)

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            print('=== Moving to Push Button ===')
            move(robot, -0.02)

            print('=== Pausing for 1 second ===')
            time.sleep(1.0)

            button_and_adjust.run(robot)

            print('=== Going to Pick Up Wafer ===')
            move(robot, -FROM_MACHINE_TO_WTABLE)
            rotate(robot, QUARTER_CLOCK)

            print('=== Pausing for 1 sec ===')
            time.sleep(1.0)

            print('=== Aligning with Wafer Station ==')
            station_navigation.run(robot, 'wafer_station', horizontal_align=False)

            rotate(robot, QUARTER_COUNTERCLOCK)

            station_navigation.run(robot, 'wafer_station', horizontal_align=True)

            print('=== Picking Up Wafer ===')
            do_dw_pupd(robot, False, False)

            print('=== Moving to Tray ===')
            station_navigation.run(robot, 'tray', horizontal_align=False)

            rotate(robot, QUARTER_COUNTERCLOCK)

            print('=== Aligning with Tray ===')
            station_navigation.run(robot, 'tray', horizontal_align=True)

            print('=== Depositing Wafer ===')
            do_dw_pupd(robot, True, True)

            print('=== Moving to Machine ===')
            move(robot, 0.9)
            rotate(robot, QUARTER_CLOCK)
            move(robot, FROM_TRAY_TO_MACHINE-0.1)

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            move(robot, 0.12)

            print('=== Closing the Machine ===')
            twist_and_adjust.run(robot, op='close')
                
            move(robot, -0.25)

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            print('=== Moving to Push Button ===')
            move(robot, -0.02)

            print('=== Pausing for 1 second ===')
            time.sleep(1.0)

            button_and_adjust.run(robot)

            # HEILUHWUHF
            
            time.sleep(5) # simulate waiting for recipe to run-- TODO: extend by having stretch dock and undock

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            move(robot, 0.12)

            print('=== Opening the Machine ===')
            twist_and_adjust.run(robot, op='open')
                
            move(robot, -0.2)

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            print('=== Moving to Push Button ===')
            move(robot, -0.02)

            print('=== Pausing for 1 second ===')
            time.sleep(1.0)

            button_and_adjust.run(robot)

            print('=== Moving to Tray ===')
            move(robot, -FROM_TRAY_TO_MACHINE-0.5)
            rotate(robot, QUARTER_COUNTERCLOCK)
            move(robot, -0.9)
            rotate(robot, QUARTER_CLOCK)

            print('=== Moving to Tray ===')
            station_navigation.run(robot, 'tray', horizontal_align=False)

            rotate(robot, QUARTER_COUNTERCLOCK)

            print('=== Aligning with Tray ===')
            station_navigation.run(robot, 'tray', horizontal_align=True)

            print('=== Withdrawing Wafer ===')
            do_dw_pupd(robot, True, False)

            print('=== Moving to Wafer Station ===')
            move(robot, 0.83)
            rotate(robot, QUARTER_CLOCK)
            move(robot, -FROM_WTABLE_TO_TRAY)
            rotate(robot, QUARTER_CLOCK)

            print('=== Going to with Wafer Station ==')
            station_navigation.run(robot, 'wafer_station', horizontal_align=False)

            rotate(robot, QUARTER_COUNTERCLOCK)

            print('=== Aligning with Wafer Station ==')
            station_navigation.run(robot, 'wafer_station', horizontal_align=True)

            print('=== Putting Down Wafer ===')
            do_dw_pupd(robot, False, True)

            print('=== Going to Machine ===')
            rotate(robot, QUARTER_COUNTERCLOCK)
            move(robot, 1.05)
            rotate(robot, QUARTER_CLOCK)
            move(robot, FROM_MACHINE_TO_WTABLE-0.05)

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            move(robot, 0.12)

            print('=== Closing the Machine ===')
            twist_and_adjust.run(robot, op='close')
                
            move(robot, -0.25)

            print('=== Aligning with Machine ===')
            base_alignment.run(robot)

            print('=== Moving to Push Button ===')
            move(robot, -0.02)

            print('=== Pausing for 1 second ===')
            time.sleep(1.0)

            button_and_adjust.run(robot)

            print('=== Complete Demo Done! ===')
            move(robot, -0.25)
        
    finally:
        if robot is not None:
            robot.stop()


if __name__ == '__main__':
    main()