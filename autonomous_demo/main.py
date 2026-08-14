"""
Complete MVP Autonomous Demo Script for the Stretch3

Demo to be extended by starting at finishing at Stretch3 charging dock
"""

import time
import math
import stretch_body.robot as rb
import d405_helpers as dh
import d435_helpers as d435h
import twist_and_adjust
import button_and_adjust
import base_alignment
import station_navigation

# coarse grain navigation and orientation constants
FROM_MACHINE_TO_WTABLE = 2
QUARTER_COUNTERCLOCK = math.pi/2
QUARTER_CLOCK = -QUARTER_COUNTERCLOCK
FROM_WTABLE_TO_TRAY = 1
FROM_TRAY_TO_MACHINE = 1.0

# debug option constants
DEBUG_BUTTON = 1
DEBUG_MACHINE_OP = 2
DEBUG_TRAY = 3

def default_gripper(robot):
    """Helper function for homing/resetting the gripper.

    Args:
        robot: the current ``stretch_body.robot.Robot`` instance.
    """
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.end_of_arm.move_to('wrist_roll', 0.0)
    robot.end_of_arm.move_to('stretch_gripper', 0) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()

def move(robot, distance: float):
    """Helper function to move the robot forward by ``distance`` meters.

    Args:
        robot: the current ``stretch_body.robot.Robot`` instance.
        distance: amount to translate the robot forward (in meters).
    """
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.base.translate_by(distance, v_m=1.0)
    robot.push_command()
    # status print statement to check whether robot completed command or timed out
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")

def rotate(robot, angle: float):
    """Helper function to rotate the robot anti-clockwise by ``angle`` radians.

    Args:
        robot: the current ``stretch_body.robot.Robot`` instance.
        angle: amount to rotate the robot by (in radians).
    """
    robot.base.rotate_by(angle)
    robot.push_command()
    # status print statement to check whether robot completed command or timed out
    print(f"waited for move by {angle}m command? {robot.wait_command()}")

def wafer_action(robot, at_tray=True, put_down=False):
    """Perform either a wafer deposit/withdraw or a pick-up/put-down.

    Robot must be aligned with the manipulation point (e.g. RIE80 tray or wafer station).

    Args:
        robot: the current ``stretch_body.robot.Robot`` instance.
        at_tray: wafer manipulation location flag; if ``True`` then RIE80 tray is the target,
                else if ``False`` then wafer station is the target.
        put_down: if ``True`` then robot will release the wafer (actuate the vacuum pen), else
                  the robot will simply pick up the wafer (no actuation, just vertical movement).
    """

    # By default initialise offsets to RIE80 tray values (in meters).
    HEIGHT = 1.1 # lift height
    LENGTH = 0.46 # arm length
    DELTA_HEIGHT = 0.05 # vertical movement amount

    if not at_tray:
        # Wafer station pose.
        HEIGHT = 0.98
        LENGTH = 0.05
        DELTA_HEIGHT = 0.11

    print(f"{HEIGHT}, {LENGTH}, {DELTA_HEIGHT}")

    # Move the wrist, lift, and arm over the respective manipulation target.
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.lift.move_to(HEIGHT)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', 0.0)
    # wrist movement is non-blocking, use sleep to ensure full movement occurs
    time.sleep(2)

    robot.arm.move_to(LENGTH)
    robot.push_command()
    robot.wait_command()

    # Lower the lift to contact the wafer
    robot.lift.move_by(-DELTA_HEIGHT, v_m=0.03)
    robot.push_command()
    robot.wait_command()

    if put_down:
        robot.end_of_arm.move_to('stretch_gripper', -70) # [-100, 100] => [fully closed, fully open]
        # wrist movement is non-blocking, use sleep to ensure full movement occurs
        time.sleep(2.5)

    # Raise arm, open gripper, and retract arm
    robot.lift.move_by(DELTA_HEIGHT)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('stretch_gripper', 25) # [-100, 100] => [fully closed, fully open]
    robot.push_command()
    robot.wait_command()

    robot.arm.move_to(0.0)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)

def debug_test(robot, option: int, num_iterations: int):   
    """Helper function to debug specific parts of the routine.

    Enter and comment respective subroutine test functions based on
    iteration segment being debugged.

    Args:
        robot: the current ``stretch_body.robot.Robot`` instance.
    """
    for i in range (0, num_iterations):
        if option == DEBUG_BUTTON:
            button_op_test(robot)
        elif option == DEBUG_MACHINE_OP:
            machine_op_debug(robot)
        elif option == DEBUG_TRAY:
            go_to_tray_debug(robot)
    return

def go_to_tray_debug(robot):
    print('=== Moving to Tray ===')
    station_navigation.run(robot, 'tray', horizontal_align=False)

    rotate(robot, QUARTER_COUNTERCLOCK)

    print('=== Aligning with Tray ===')
    station_navigation.run(robot, 'tray', horizontal_align=True)

    print('=== Depositing Wafer ===')
    wafer_action(robot, True, True)
                        

def machine_op_debug(robot):
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
    print('=== Aligning with Machine ===')
    base_alignment.run(robot)

    print('=== Moving to Push Button ===')
    move(robot, -0.02)

    print('=== Pausing for 1 second ===')
    time.sleep(1.0)

    button_and_adjust.run(robot)
        
    move(robot, -0.15)



def main():
    """Start the robot, run one demo iteration, and stop on exit.

    The normal path performs the complete wafer-loading and unloading sequence.
    Setting the local ``DEBUG`` flag selects the partial debug routine instead.
    Robot shutdown is attempted in the ``finally`` block after the run.
    """
    try:
        robot = rb.Robot()
        robot.startup()

        # TODO: change to use argparse/CLI input instead (extend to debug and num iterations as well)
        DEBUG = False 

        if DEBUG:
            debug_test(robot, DEBUG_BUTTON, 1)
            return
        
        i = 0

        while i < 1:
            i += 1

            print(f'=== iteration: {i} ===\n\n')

            # Phase 1: Align with the RIE80, open it, and operate its button.
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

            # Phase 2: Move to the wafer station and pick up the wafer.
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
            wafer_action(robot, False, False)

            # Phase 3: Move to the RIE80 tray and deposit the wafer.
            print('=== Moving to Tray ===')
            station_navigation.run(robot, 'tray', horizontal_align=False)

            rotate(robot, QUARTER_COUNTERCLOCK)

            print('=== Aligning with Tray ===')
            station_navigation.run(robot, 'tray', horizontal_align=True)

            print('=== Depositing Wafer ===')
            wafer_action(robot, True, True)

            # Phase 4: Return to the controls and close the RIE80.
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
            
            time.sleep(5) # simulate waiting for recipe to run-- TODO: extend by having stretch dock and undock

            # Phase 5: Reopen the RIE80 before retrieving the wafer.
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

            # Phase 6: Return to the tray and withdraw the wafer.
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
            wafer_action(robot, True, False)

            # Phase 7: Return the wafer to the wafer station.
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
            wafer_action(robot, False, True)

            # Phase 8: Return to the RIE80 and leave it closed.
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
