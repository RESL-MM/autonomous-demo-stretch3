import stretch_body.robot as rb
import math
import argparse
import time

def move(robot, distance: float):
    robot.base.translate_by(distance, v_m=0.25)
    robot.push_command()
    print(f"waited for move by {distance}m command? {robot.wait_command(timeout=60.0)}")

def rotate(robot, angle: float):
    robot.base.rotate_by(angle)
    robot.push_command()
    print(f"waited for move by {angle}m command? {robot.wait_command()}")

def main(dw):
    robot = rb.Robot()
    robot.startup()

    STOW_ANGLE = (3 * math.pi)/4
    PUPD_DIST = 0.03

    # deposit/withdraw values
    DW_HEIGHT = 1.05
    DW_LENGTH = 0.3

    if not dw:
        # pick up/put down values
        DW_HEIGHT = 0.8
        DW_LENGTH = 0.3

    """
    withdraw = deposit with opposite gripper action
    assume start with arm extending out towards machine

    CHECK GRIPPER ORIENTATION-- WHEN TO STOW/UNSTOW?
    1) raise arm to correct height
    2) extend arm
    2.5) can 1 and 2 be done independently? does 2 need to go up and down to avoid the machine?
    3) lower lift to contact wafer spot
    3.5) if DEPOSIT, then RELEASE WAFER (close gripper), else do nothing
    4) retract arm
    4.5) open gripper (if not open)
    5) fully or partially stow gripper
    """
    robot.stow()

    print(f"{DW_HEIGHT}, {DW_LENGTH}, {PUPD_DIST}")
    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
    robot.end_of_arm.move_to('wrist_pitch', 0.0)
    robot.lift.move_to(DW_HEIGHT)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', 0.0)

    robot.arm.move_to(DW_LENGTH)
    robot.push_command()
    robot.wait_command()

    # robot.end_of_arm.move_to(-50) # [-100, 100] => [fully closed, fully open]
    # robot.push_command()
    # robot.wait_command()
    
    robot.lift.move_by(-PUPD_DIST)
    robot.push_command()
    robot.wait_command()

    robot.lift.move_by(PUPD_DIST)
    robot.push_command()
    robot.wait_command()

    robot.arm.move_to(0.0)
    robot.push_command()
    robot.wait_command()

    robot.end_of_arm.move_to('wrist_yaw', math.pi/2)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='Stretch3 Wafer Manipulation Tests')
    parser.add_argument('-dw', '--DW', type=str, default='dw', help='set to dw or leave empty for deposit/withdraw behaviour, else change to anything for pick-up/put-down behaviour')
    args = parser.parse_args()
    dw = True
    if args.DW != 'dw':
        dw = False

    print(dw)

    main(dw)